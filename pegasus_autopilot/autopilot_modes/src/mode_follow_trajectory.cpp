#include "autopilot_modes/mode_follow_trajectory.hpp"

namespace autopilot {

FollowTrajectoryMode::~FollowTrajectoryMode() {}

void FollowTrajectoryMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "FollowTrajectoryMode initialized");
    return;
}

bool FollowTrajectoryMode::enter() {
    // TODO: Implement this method
    return true;
}

bool FollowTrajectoryMode::exit() {
    // TODO: Implement this method
    return true;
}

void FollowTrajectoryMode::update(double) {
    // TODO: Implement this method
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)