#include "autopilot_modes/mode_land.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

LandMode::~LandMode() {}

void LandMode::initialize() {

    // Get the parameter for the landing speed
    node_->declare_parameter<float>("autopilot.LandMode.land_speed", 0.5f);
    land_speed_ = node_->get_parameter("autopilot.LandMode.land_speed").as_double();

    // Get the parameter for the landing detection treshold
    node_->declare_parameter<float>("autopilot.LandMode.land_detected_treshold", 0.1f);
    land_detected_treshold_ = node_->get_parameter("autopilot.LandMode.land_detected_treshold").as_double();

    // Log that the LandMode has been initialized successfully
    RCLCPP_INFO(this->node_->get_logger(), "LandMode initialized");
    RCLCPP_INFO(this->node_->get_logger(), "LandMode land_speed: %.2f m/s", land_speed_);
    RCLCPP_INFO(this->node_->get_logger(), "LandMode land_detected_treshold: %.2f m/s", land_detected_treshold_);
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

bool LandMode::check_land_complete(double curr_z, double prev_z, double dt) {
    
    // Check if the drone is no longer moving on the z-axis (note: we assume NED coordinates here)
    // TODO: This method for landing can be improved with a low pass filter and better checks, but for now it will do
    return (curr_z - prev_z  < land_detected_treshold_) ? true : false;
}

void LandMode::update(double dt) {

    // Get the current state of the drone
    State curr_state = this->get_vehicle_state();

    // Update the target Z position based on the current land speed
    this->target_pos_[2] = curr_state.position[2] + (this->land_speed_ * dt);

    // Set the controller to track the position which is slighlty bellow the vehicle, but keep the original orientation
    this->set_position(this->target_pos_, this->target_yaw_);

    // Check if the position is no longer changing - if so, it means that the drone has landed and we should signal the mode as finished
    if (check_land_complete(curr_state.position[2], prev_z_pos_, dt)) signal_mode_finished();

    // Set the previous position of the drone on the z-axis
    this->prev_z_pos_ = curr_state.position[2];
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::LandMode, autopilot::Mode)