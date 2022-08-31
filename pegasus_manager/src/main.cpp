#include "rclcpp/rclcpp.hpp"
#include "manager_node.hpp"

int main(int argc, char ** argv) {
    // Init ROS2
    rclcpp::init(argc, argv);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "OK0");

    // Create the Manager node object
    std::shared_ptr<ManagerNode> node = std::make_shared<ManagerNode>("manager_node");

    // Going into spin mode and letting the callbacks do all the work
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}