#include "autopilot_modes/mode_arm.hpp"

namespace autopilot {

ArmMode::~ArmMode() {}

void ArmMode::initialize() {
    // Do nothing 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ArmMode initialized");
    //RCLCPP_INFO(this->node_->get_logger(), "ArmMode initialized");
}

bool ArmMode::enter() {
    // TODO: Implement this method
    return true;
}

bool ArmMode::exit() {
    // TODO: Implement this method
    return true;
}

void ArmMode::update(double) {
    // TODO: Implement this method
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::ArmMode, autopilot::Mode)