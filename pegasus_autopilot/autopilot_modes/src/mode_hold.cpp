#include "autopilot_modes/mode_hold.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

HoldMode::~HoldMode() {}

void HoldMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "HoldMode initialized");
    return;
}

bool HoldMode::enter() {
    
    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->target_pos[0] = curr_state.position[0];
    this->target_pos[1] = curr_state.position[1];
    this->target_pos[2] = curr_state.position[2];

    // TODO: Check if we need to convert the yaw from rad to deg to be used by the target position
    this->target_yaw = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool HoldMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void HoldMode::update(double) {

    // Set the controller to track the target position and attitude
    this->set_position(this->target_pos, this->target_yaw);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::HoldMode, autopilot::Mode)