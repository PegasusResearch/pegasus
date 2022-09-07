#include "paths_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {

    // Initiate ROS 2
    rclcpp::init(argc, argv);

    // Create the Paths node object
    PathsNode::SharedPtr paths_node = std::make_shared<PathsNode>("paths_node");

    // Going into spin mode and letting the callbacks do all the work
    rclcpp::spin(paths_node);

    // Shutdown the ROS node
    rclcpp::shutdown();
    return 0;
}