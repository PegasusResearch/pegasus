#include "autopilot_modes/mode_pass_through.hpp"

namespace autopilot {

PassThroughMode::~PassThroughMode() {}

void PassThroughMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "PassThroughMode initialized");
    return;
}

bool PassThroughMode::enter() {
    // Do nothing and just return
    return true;
}

bool PassThroughMode::exit() {
    // Do nothing and just return
    return true;
}

void PassThroughMode::update(double) {
    // Do nothing and just return
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::PassThroughMode, autopilot::Mode)