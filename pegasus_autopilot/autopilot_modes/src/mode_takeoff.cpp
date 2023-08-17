#include "autopilot_modes/mode_takeoff.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

TakeoffMode::~TakeoffMode() {}

void TakeoffMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "TakeoffMode initialized");
    return;
}

bool TakeoffMode::enter() {
    
    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->takeoff_pos[0] = curr_state.position[0];
    this->takeoff_pos[1] = curr_state.position[1];
    this->takeoff_pos[2] = curr_state.position[2] - this->target_altitude;

    // TODO: Check if we need to convert the yaw from rad to deg to be used by the target position
    this->takeoff_yaw = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool TakeoffMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void TakeoffMode::update(double) {

    // Set the controller to track the target position and attitude
    this->set_position(this->takeoff_pos, this->takeoff_yaw);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::TakeoffMode, autopilot::Mode)