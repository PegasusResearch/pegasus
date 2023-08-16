#include "autopilot_modes/mode_pass_through.hpp"

namespace autopilot {

PassThroughMode::~PassThroughMode() {}

void PassThroughMode::initialize() {
    // Do nothing
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PassThroughMode initialized");
    //RCLCPP_INFO(this->node_->get_logger(), "ArmMode initialized");
    return;
}

bool PassThroughMode::enter() {
    // TODO: Implement this method
    return true;
}

bool PassThroughMode::exit() {
    // TODO: Implement this method
    return true;
}

void PassThroughMode::update(double) {
    // TODO: Implement this method
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::PassThroughMode, autopilot::Mode)