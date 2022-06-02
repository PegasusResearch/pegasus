#include "rclcpp/rclcpp.hpp"
#include "mavlink_node.hpp"

MAVLinkNode::MAVLinkNode() : Node("mavlink_node") {
    // TODO
}

MAVLinkNode::~MAVLinkNode() {
    // TODO
}

int main(int argc, char ** argv) {

    // Initiate ROS2 library
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);

    // Create a node to send and receive data between MAVLink and ROS2
    auto node = std::make_shared<rclcpp::Node>("mavros_node", options);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}