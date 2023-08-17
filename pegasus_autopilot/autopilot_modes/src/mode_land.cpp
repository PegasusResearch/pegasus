#include "autopilot_modes/mode_land.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

LandMode::~LandMode() {}

void LandMode::initialize() {
    // Do nothing
    RCLCPP_INFO(this->node_->get_logger(), "LandMode initialized");
    return;
}

bool LandMode::enter() {

    // Get the current state of the drone
    State curr_state = this->get_vehicle_state();

    // Save the target X and Y position of the drone as well as the yaw to keep
    this->target_pos_[0] = curr_state.position[0];
    this->target_pos_[1] = curr_state.position[1];
    this->target_pos_[2] = curr_state.position[2];

    // TODO: Check if we need to convert the yaw from rad to deg to be used by the target position
    this->target_yaw_ = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool LandMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void LandMode::update(double dt) {

    // Get the current state of the drone
    State curr_state = this->get_vehicle_state();

    // Update the target Z position based on the current land speed
    this->target_pos_[2] = curr_state.position[2] + (this->land_speed_ * dt);

    // Set the controller to track the position which is slighlty bellow the vehicle, but keep the original orientation
    this->set_position(this->target_pos_, this->target_yaw_);

    // TODO: Check if the position is no longer changing - if so, it means that the drone has landed
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::LandMode, autopilot::Mode)