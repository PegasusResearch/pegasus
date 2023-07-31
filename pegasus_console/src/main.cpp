#include "rclcpp/rclcpp.hpp"
#include "console_node.hpp"

int main(int argc, char * argv[]) {
    
    rclcpp::init(argc, argv);

    // Create the ConsoleNode
    auto console_node = std::make_shared<ConsoleNode>();
    console_node->start();

    rclcpp::shutdown();
    return 0;
}