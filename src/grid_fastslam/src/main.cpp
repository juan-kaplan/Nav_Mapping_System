#include "rclcpp/rclcpp.hpp"
#include "grid_fastslam/grid_fastslam.hpp"

int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<grid_fastslam::GridFastSlam>();

    // Spin the node (keep it alive)
    rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();
    return 0;
}