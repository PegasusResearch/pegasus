#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "autopilot/autopilot.hpp"

int main(int argc, char ** argv) {
    
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the autopilot node
    auto autopilot_node = std::make_shared<autopilot::Autopilot>();

    // Initialize the autopilot node
    autopilot_node->initialize();

    // Spin the node until shutdown
    rclcpp::spin(autopilot_node);
    rclcpp::shutdown();
    return 0;
}
