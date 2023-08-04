#include "autopilot.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pegasus::Autopilot>());
    rclcpp::shutdown();
    return 0;
}