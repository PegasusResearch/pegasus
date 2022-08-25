#include "pid_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {

    // Initiate ROS 2
    rclcpp::init(argc, argv);

    // Create the PID node object
    std::shared_ptr<PidNode> pid_node = std::make_shared<PidNode>("pid_node");

    // Going into spin mode and letting the callbacks do all the work
    rclcpp::spin(pid_node->get_node_base_interface());

    // Shutdown the ROS node
    rclcpp::shutdown();
    return 0;
}