#include "rclcpp/rclcpp.hpp"
#include "ros_node.hpp"

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

    // Create the ros_node object that will receive the nodehandler by reference
    // and initialize the ros2 communication side
    auto ros_node = std::make_shared<ROSNode>();
    ros_node->start();

    rclcpp::shutdown();
    return 0;
}