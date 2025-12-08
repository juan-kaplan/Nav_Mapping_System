#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_msgs/msg/delta_odom.hpp"
#include <cmath>
#include <deque>

using namespace std::chrono_literals;

class DeltaOdomNode : public rclcpp::Node {
public:
    DeltaOdomNode() : Node("delta_odom_node") {
        // Subscribers - Keep queue small to ensure we always read *newest* data
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/calc_odom", qos, std::bind(&DeltaOdomNode::odom_callback, this, std::placeholders::_1));

        sub_real_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&DeltaOdomNode::real_odom_callback, this, std::placeholders::_1));

        pub_calc_path_ = this->create_publisher<nav_msgs::msg::Path>("/calc_robot_path", 10);
        pub_real_path_ = this->create_publisher<nav_msgs::msg::Path>("/real_robot_path", 10);
        pub_delta_ = this->create_publisher<custom_msgs::msg::DeltaOdom>("/delta", 10);

        calc_path_msg_.header.frame_id = "map";
        real_path_msg_.header.frame_id = "map";

        publish_timer_ = this->create_wall_timer(
            33ms, std::bind(&DeltaOdomNode::publish_loop, this)); // ~30Hz
    }

private:
    // Accumulators for motion
    double pending_dx_ = 0.0;
    double pending_dy_ = 0.0;
    double pending_dtheta_ = 0.0;
    
    // Last state state
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_theta_ = 0.0;
    bool read_odom_ = false;

    // Visualization buffers
    const size_t MAX_PATH_SIZE = 1500; 
    geometry_msgs::msg::PoseStamped latest_calc_pose_;
    geometry_msgs::msg::PoseStamped latest_real_pose_;
    bool new_calc_pose_ = false;
    bool new_real_pose_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_real_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_calc_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_real_path_;
    rclcpp::Publisher<custom_msgs::msg::DeltaOdom>::SharedPtr pub_delta_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    nav_msgs::msg::Path calc_path_msg_;
    nav_msgs::msg::Path real_path_msg_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, theta;
        tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

        if (!read_odom_) {
            last_x_ = x;
            last_y_ = y;
            last_theta_ = theta;
            read_odom_ = true;
            return;
        }

        // Calculate increment from last message
        double dx = x - last_x_;
        double dy = y - last_y_;
        
        // Handle angle wrap-around for raw theta
        double dtheta = theta - last_theta_;
        if (dtheta > M_PI) dtheta -= 2 * M_PI;
        if (dtheta < -M_PI) dtheta += 2 * M_PI;

        // ACCUMULATE the motion
        pending_dx_ += dx;
        pending_dy_ += dy;
        pending_dtheta_ += dtheta;

        // Update baseline
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;

        // Save pose for the timer to visualize later
        latest_calc_pose_.header = msg->header;
        latest_calc_pose_.pose = msg->pose.pose;
        new_calc_pose_ = true;
    }

    void real_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_real_pose_.header = msg->header;
        latest_real_pose_.pose = msg->pose.pose;
        new_real_pose_ = true;
    }

    void publish_loop() {
        double delta_t = std::sqrt(pending_dx_ * pending_dx_ + pending_dy_ * pending_dy_);
        
        if (delta_t > 1e-6 || std::abs(pending_dtheta_) > 1e-6) {
            double delta_rot1 = 0.0;
            double delta_rot2 = 0.0;

            if (delta_t > 1e-6) {
                double heading = std::atan2(pending_dy_, pending_dx_);
                double trans_angle = std::atan2(pending_dy_, pending_dx_);
                double start_theta = last_theta_ - pending_dtheta_; 

                delta_rot1 = trans_angle - start_theta;
                delta_rot2 = pending_dtheta_ - delta_rot1;
            } else {
                delta_rot1 = 0.0;
                delta_rot2 = pending_dtheta_;
            }

            // Normalize
            delta_rot1 = std::atan2(std::sin(delta_rot1), std::cos(delta_rot1));
            delta_rot2 = std::atan2(std::sin(delta_rot2), std::cos(delta_rot2));

            custom_msgs::msg::DeltaOdom delta_msg;
            delta_msg.dr1 = delta_rot1;
            delta_msg.dr2 = delta_rot2;
            delta_msg.dt = delta_t;
            pub_delta_->publish(delta_msg);

            // Reset accumulators
            pending_dx_ = 0.0;
            pending_dy_ = 0.0;
            pending_dtheta_ = 0.0;
        }

        // --- B. Visualization (Calc Path) ---
        if (new_calc_pose_) {
            calc_path_msg_.header.stamp = latest_calc_pose_.header.stamp;
            calc_path_msg_.poses.push_back(latest_calc_pose_);
            
            if (calc_path_msg_.poses.size() > MAX_PATH_SIZE) {
                calc_path_msg_.poses.erase(calc_path_msg_.poses.begin());
            }
            pub_calc_path_->publish(calc_path_msg_);
            new_calc_pose_ = false;
        }

        // --- C. Visualization (Real Path) ---
        if (new_real_pose_) {
            real_path_msg_.header.stamp = latest_real_pose_.header.stamp;
            real_path_msg_.poses.push_back(latest_real_pose_);
            
            if (real_path_msg_.poses.size() > MAX_PATH_SIZE) {
                real_path_msg_.poses.erase(real_path_msg_.poses.begin());
            }
            pub_real_path_->publish(real_path_msg_);
            new_real_pose_ = false;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeltaOdomNode>());
    rclcpp::shutdown();
    return 0;
}