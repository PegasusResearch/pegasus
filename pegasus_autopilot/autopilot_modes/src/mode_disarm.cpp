#include "autopilot_modes/mode_disarm.hpp"

namespace autopilot {

DisarmMode::~DisarmMode() {}

void DisarmMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "DisarmMode initialized");
    return;
}

bool DisarmMode::enter() {
    // TODO: Implement this method
    return true;
}

bool DisarmMode::exit() {
    // TODO: Implement this method
    return true;
}

void DisarmMode::update(double) {
    // TODO: Implement this method
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::DisarmMode, autopilot::Mode)