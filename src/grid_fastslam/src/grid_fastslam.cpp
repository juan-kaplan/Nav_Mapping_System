#include "grid_fastslam/grid_fastslam.hpp"

namespace grid_fastslam
{

    GridFastSlam::GridFastSlam()
        : Node("grid_fast_slam_node"),
          rng_(std::random_device{}())
    {
        this->declare_parameter("num_particles", 10);
        num_particles_ = this->get_parameter("num_particles").as_int();

        particles_.resize(num_particles_);

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.reliable();
        map_qos.transient_local();

        delta_sub_ = this->create_subscription<custom_msgs::msg::DeltaOdom>(
            "/delta", 10,
            std::bind(&GridFastSlam::delta_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GridFastSlam::scan_callback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/particle_robot_path", 10);

        path_msg_.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "GridFastSlam initialized with %d particles.", num_particles_);
    }

    void GridFastSlam::delta_callback(const custom_msgs::msg::DeltaOdom::SharedPtr msg)
    {
        double dr1 = msg->dr1;
        double dr2 = msg->dr2;
        double dt = msg->dt;

        if (std::abs(dr1) < 1e-6 && std::abs(dr2) < 1e-6 && std::abs(dt) < 1e-6)
        {
            return;
        }

        this->move_particles(dr1, dr2, dt);
    }

    void GridFastSlam::move_particles(double dr1, double dr2, double dt)
    {
        const double alpha1 = 0.05;
        const double alpha2 = 0.05;
        const double alpha3 = 0.001;
        const double alpha4 = 0.001;

        double sigma_rot1 = alpha1 * std::abs(dr1) + alpha2 * dt;
        double sigma_trans = alpha3 * dt + alpha4 * (std::abs(dr1) + std::abs(dr2));
        double sigma_rot2 = alpha1 * std::abs(dr2) + alpha2 * dt;

        std::normal_distribution<double> noise_rot1(0.0, sigma_rot1);
        std::normal_distribution<double> noise_trans(0.0, sigma_trans);
        std::normal_distribution<double> noise_rot2(0.0, sigma_rot2);

        for (auto &p : particles_)
        {
            double rot1_hat = dr1 + noise_rot1(rng_);
            double trans_hat = dt + noise_trans(rng_);
            double rot2_hat = dr2 + noise_rot2(rng_);

            p.x += trans_hat * std::cos(p.yaw + rot1_hat);
            p.y += trans_hat * std::sin(p.yaw + rot1_hat);
            p.yaw += rot1_hat + rot2_hat;

            p.yaw = normalize_angle(p.yaw);
        }
    }

    double GridFastSlam::normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // =========================================================
    // HELPER FUNCTIONS (COORDINATES & RAYCASTING)
    // =========================================================

    std::pair<int, int> GridFastSlam::world_to_grid(double x, double y)
    {
        // Convert World (meters) to Map (indices)
        // Python: j = floor((x - ox) / res), i = floor((y - oy) / res)
        int j = std::floor((x - OX) / RESOLUTION); // x is column (width)
        int i = std::floor((y - OY) / RESOLUTION); // y is row (height)
        return {i, j};
    }

    std::vector<std::pair<int, int>> GridFastSlam::bresenham(int i0, int j0, int i1, int j1)
    {
        // The "Line Drawing" Algorithm
        // This connects two grid cells with the pixels in between
        std::vector<std::pair<int, int>> cells;

        int di = std::abs(i1 - i0);
        int dj = std::abs(j1 - j0);
        int si = (i0 < i1) ? 1 : -1;
        int sj = (j0 < j1) ? 1 : -1;
        int err = di - dj;

        int i = i0;
        int j = j0;

        while (true)
        {
            cells.push_back({i, j});
            if (i == i1 && j == j1)
                break;
            int e2 = 2 * err;
            if (e2 > -dj)
            {
                err -= dj;
                i += si;
            }
            if (e2 < di)
            {
                err += di;
                j += sj;
            }
        }
        return cells;
    }

    // =========================================================
    // GRID UPDATE LOGIC
    // =========================================================

    // Internal helper to calculate where laser points land in the world
    // Optimization: Only process points within +/- 90 degrees (Front view)
    std::vector<std::pair<double, double>> get_scan_endpoints(
        const Particle &p,
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<std::pair<double, double>> points;
        double raw_angle = scan->angle_min;
        double limit_max = scan->range_max - 0.001;

        // Pre-compute particle rotation
        double sin_p = std::sin(p.yaw);
        double cos_p = std::cos(p.yaw);

        // Define Field of View Limits (-PI/2 to +PI/2)
        const double angle_min_limit = -M_PI_2; // -90 degrees
        const double angle_max_limit = M_PI_2;  // +90 degrees

        for (size_t k = 0; k < scan->ranges.size(); ++k)
        {
            double r = scan->ranges[k];

            // --- NEW: Normalize angle to [-PI, PI] ---
            double angle_wrapped = raw_angle;
            while (angle_wrapped > M_PI)
                angle_wrapped -= 2.0 * M_PI;
            while (angle_wrapped < -M_PI)
                angle_wrapped += 2.0 * M_PI;

            // 1. FILTER: Now this works for both sides!
            if (angle_wrapped >= angle_min_limit && angle_wrapped <= angle_max_limit)
            {

                // 2. VALIDITY: Check if range is valid
                if (std::isfinite(r) && r > scan->range_min && r < limit_max)
                {

                    // Polar -> Cartesian (Scanner Frame)
                    // Note: Use raw_angle (or wrapped, sin/cos are same)
                    double lx = r * std::cos(raw_angle);
                    double ly = r * std::sin(raw_angle);

                    // Rotation + Translation (World Frame)
                    double wx = (cos_p * lx - sin_p * ly) + p.x;
                    double wy = (sin_p * lx + cos_p * ly) + p.y;

                    points.push_back({wx, wy});
                }
            }

            // Always increment the RAW angle tracker
            raw_angle += scan->angle_increment;
        }
        return points;
    }

    void GridFastSlam::update_grid(Particle &p, const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 1. Get where the laser hits are in the real world
        auto endpoints = get_scan_endpoints(p, scan);
        if (endpoints.empty())
            return;

        // 2. Get Robot's position in the grid
        auto [r0, c0] = world_to_grid(p.x, p.y);
        if (r0 < 0 || r0 >= MAP_HEIGHT || c0 < 0 || c0 >= MAP_WIDTH)
            return;

        // 3. Loop through rays
        int beam_step = 5;

        for (size_t k = 0; k < endpoints.size(); k += beam_step)
        {
            auto [wx, wy] = endpoints[k];
            auto [r1, c1] = world_to_grid(wx, wy);

            // Check if endpoint is inside map
            if (r1 < 0 || r1 >= MAP_HEIGHT || c1 < 0 || c1 >= MAP_WIDTH)
                continue;

            // 4. Raycast (Bresenham)
            // Get all cells between robot and hit point
            auto ray = bresenham(r0, c0, r1, c1);

            // A. Free Space (All cells EXCEPT the last one)
            for (size_t j = 0; j < ray.size() - 1; ++j)
            {
                int fr = ray[j].first;
                int fc = ray[j].second;

                // Boundary check (safe programming)
                if (fr >= 0 && fr < MAP_HEIGHT && fc >= 0 && fc < MAP_WIDTH)
                {
                    int idx = fr * MAP_WIDTH + fc; // 2D -> 1D Index
                    p.grid[idx] += L_FREE;
                    if (p.grid[idx] < L_MIN)
                        p.grid[idx] = L_MIN; // Clamp
                }
            }

            // B. Occupied Space (The last cell where the laser hit)
            int idx_hit = r1 * MAP_WIDTH + c1;
            p.grid[idx_hit] += L_OCC;
            if (p.grid[idx_hit] > L_MAX)
                p.grid[idx_hit] = L_MAX; // Clamp
        }
    }

    // =========================================================
    // PROBABILITY & RESAMPLING LOGIC
    // =========================================================

    // Helper: Numerical stability for normalizing weights
    // Computes log(sum(exp(x_i))) without overflowing
    double log_sum_exp(const std::vector<double> &log_weights)
    {
        if (log_weights.empty())
            return -std::numeric_limits<double>::infinity();

        // 1. Find the maximum value to shift the range
        double max_log_w = *std::max_element(log_weights.begin(), log_weights.end());

        // 2. Compute sum of exponentials of the differences
        double sum = 0.0;
        for (double lw : log_weights)
        {
            sum += std::exp(lw - max_log_w);
        }

        // 3. Return result in log domain
        return max_log_w + std::log(sum);
    }

    void GridFastSlam::update_particles(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::vector<double> log_weights(num_particles_);
        bool any_valid_weight = false;

        // 1. Calculate weight for EACH particle
        for (int i = 0; i < num_particles_; ++i)
        {
            auto &p = particles_[i];

            // Get valid scan points in World Frame
            auto endpoints = get_scan_endpoints(p, scan);

            // If no valid points, this particle is dead
            if (endpoints.empty())
            {
                log_weights[i] = -1.0e9;
                continue;
            }

            double log_w_sum = 0.0;
            int hit_count = 0;

            // Compare scan to the particle's OWN map
            for (auto &pt : endpoints)
            {
                auto [r, c] = world_to_grid(pt.first, pt.second);

                // Check if point is inside the map
                if (r >= 0 && r < MAP_HEIGHT && c >= 0 && c < MAP_WIDTH)
                {
                    // Get the Log-Odds value from the grid
                    float log_odds = p.grid[r * MAP_WIDTH + c];

                    // Inverse Sensor Model: Convert Log-Odds -> Probability
                    // p = 1 / (1 + exp(-l))
                    double prob = 1.0 / (1.0 + std::exp(-log_odds));

                    // Clip for stability (same as Python 1e-6)
                    prob = std::max(1e-6, std::min(prob, 1.0 - 1e-6));

                    // Accumulate log-likelihood
                    log_w_sum += std::log(prob);
                    hit_count++;
                }
            }

            if (hit_count > 0)
            {
                // Average log-likelihood (Geometric Mean of probabilities)
                log_weights[i] = log_w_sum / hit_count;
                any_valid_weight = true;
            }
            else
            {
                log_weights[i] = -1.0e9;
            }
        }

        // 2. Normalize Weights (LogSumExp Trick)
        if (!any_valid_weight)
        {
            // If all particles failed, reset to uniform distribution
            for (auto &p : particles_)
                p.weight = 1.0 / num_particles_;
        }
        else
        {
            // Calculate the normalizing constant
            double log_sum = log_sum_exp(log_weights);

            // Update linear weights: w = exp(log_w - log_sum)
            for (int i = 0; i < num_particles_; ++i)
            {
                particles_[i].weight = std::exp(log_weights[i] - log_sum);
            }
        }
    }

    void GridFastSlam::resample()
    {
        // 1. Calculate Effective Sample Size (N_eff)
        double sum_sq = 0.0;
        for (const auto &p : particles_)
        {
            sum_sq += (p.weight * p.weight);
        }
        double n_eff = 1.0 / (sum_sq + 1e-9);

        // 2. Resample only if particles have degraded (N_eff < 50%)
        if (n_eff < (num_particles_ * 0.5))
        {

            std::vector<Particle> new_particles;
            new_particles.reserve(num_particles_);

            // --- Systematic Resampling (Low Variance Sampling) ---
            // This is the standard algorithm for particle filters

            std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
            double r = dist(rng_);           // Random start point
            double c = particles_[0].weight; // Cumulative weight
            int i = 0;

            for (int m = 0; m < num_particles_; ++m)
            {
                double u = r + (double)m / num_particles_; // Step through the wheel

                // Find the particle that spans this weight segment
                while (u > c && i < num_particles_ - 1)
                {
                    i++;
                    c += particles_[i].weight;
                }

                // Copy the particle
                Particle p_copy = particles_[i];
                p_copy.weight = 1.0 / num_particles_; // Reset weight after resampling
                new_particles.push_back(p_copy);
            }

            // Replace old set with new set
            particles_ = new_particles;
            // RCLCPP_INFO(this->get_logger(), "Resampled! N_eff: %.2f", n_eff);
        }
    }

    void GridFastSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. Update Weights based on how well scan matches map
        update_particles(msg);

        // 2. Resample (Evolution) - Kill bad particles, multiply good ones
        resample();

        // 3. Update Map (Learning) - Update the grid of every particle
        for (auto &p : particles_)
        {
            update_grid(p, msg);
        }

        // 4. Publish Visualization
        publish_map_and_path();
    }

    void GridFastSlam::publish_map_and_path()
    {
        // Find the single best particle to visualize
        auto best_it = std::max_element(particles_.begin(), particles_.end(),
                                        [](const Particle &a, const Particle &b)
                                        { return a.weight < b.weight; });

        const auto &best_p = *best_it;

        // --- Publish Map ---
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = RESOLUTION;
        map_msg.info.width = MAP_WIDTH;
        map_msg.info.height = MAP_HEIGHT;
        map_msg.info.origin.position.x = OX;
        map_msg.info.origin.position.y = OY;

        // Orientation must be valid quaternion (0,0,0,1)
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data.resize(MAP_WIDTH * MAP_HEIGHT);

        // Convert Log-Odds to Int8 [0-100] for RViz
        for (size_t i = 0; i < best_p.grid.size(); ++i)
        {
            // Probability p = 1 / (1 + exp(-l))
            double p = 1.0 / (1.0 + std::exp(-best_p.grid[i]));
            map_msg.data[i] = static_cast<int8_t>(p * 100);
        }
        map_pub_->publish(map_msg);

        // --- Publish Path ---
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map_msg.header;
        pose.pose.position.x = best_p.x;
        pose.pose.position.y = best_p.y;

        // Yaw to Quaternion (Simple Z-axis rotation)
        pose.pose.orientation.z = std::sin(best_p.yaw * 0.5);
        pose.pose.orientation.w = std::cos(best_p.yaw * 0.5);

        path_msg_.header.stamp = this->now();
        path_msg_.poses.push_back(pose);
        path_pub_->publish(path_msg_);
    }

} // namespace grid_fastslam