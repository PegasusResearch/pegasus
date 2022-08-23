#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "mavlink_node.hpp"
#include "ros_node.hpp"
#include <thread>

/**
 * @brief The entrypoint of the mavlink driver node
 * 
 * @param argc The number of arguments passed to the program
 * @param argv The argument string array passed to the program
 * @return int The execution error returned to the user
 */
int main(int argc, char ** argv) {

    // Initiate ROS2 library
    rclcpp::init(argc, argv);

    // Create a ROS2 generic Node handler that will be passed to the mavlink_node and ros_node objects
    // so that they can communicate to ROS
    const std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("mavlink_driver_node");

    // Create the ros_node object that will receive the nodehandler by reference
    // and initialize the ros2 communication side
    const std::shared_ptr<ROSNode> ros_node = std::make_shared<ROSNode>(nh);

    // Create a mavlink_node object that will receive the nodehandler by reference
    // and initialize the mavlink communication side
    const std::shared_ptr<MavlinkNode> mavlink_node = std::make_shared<MavlinkNode>(nh, ros_node);

    // Add the pointer of ros_node to the mavlink_node (initialize the circular dependency between the 2 objects)
    ros_node->init_mavlink_node(mavlink_node);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    
    return 0;
}