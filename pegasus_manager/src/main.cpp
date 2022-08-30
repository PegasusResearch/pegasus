#include "manager_node.hpp"

int main(int argc, char ** argv) {
    // Init ROS2
    rclcpp::init(argc, argv);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}