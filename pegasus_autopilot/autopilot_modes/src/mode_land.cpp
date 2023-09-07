/****************************************************************************
 *
 *   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Pegasus nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "autopilot_modes/mode_land.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

LandMode::~LandMode() {}

void LandMode::initialize() {

    // Get the parameter for the landing speed
    node_->declare_parameter<float>("autopilot.LandMode.land_speed", 0.1f);
    land_speed_ = node_->get_parameter("autopilot.LandMode.land_speed").as_double();

    // Get the parameter for the landing detection treshold
    node_->declare_parameter<float>("autopilot.LandMode.land_detected_treshold", 0.06f);
    land_detected_treshold_ = node_->get_parameter("autopilot.LandMode.land_detected_treshold").as_double();

    // Get the parameter for the switch to disarm
    node_->declare_parameter<float>("autopilot.LandMode.countdown_to_disarm", 5.0f);
    countdown_to_disarm_ = node_->get_parameter("autopilot.LandMode.countdown_to_disarm").as_double();

    // Log that the LandMode has been initialized successfully
    RCLCPP_INFO(this->node_->get_logger(), "LandMode initialized");
    RCLCPP_INFO(this->node_->get_logger(), "LandMode land_speed: %.2f m/s", land_speed_);
    RCLCPP_INFO(this->node_->get_logger(), "LandMode land_detected_treshold: %.2f m/s", land_detected_treshold_);
}

bool LandMode::enter() {

    // Reset the land counter
    this->land_counter_ = countdown_to_disarm_;

    // Get the current state of the drone
    State curr_state = this->get_vehicle_state();

    // Save the target X and Y position of the drone as well as the yaw to keep
    this->target_pos_[0] = curr_state.position[0];
    this->target_pos_[1] = curr_state.position[1];
    this->target_pos_[2] = curr_state.position[2];

    // Convert the yaw from rad to deg to be used by the target position
    this->target_yaw_ = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool LandMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

bool LandMode::check_land_complete(double velocity_z, double dt) {
    
    // Check if the drone is no longer moving on the z-axis (note: we assume NED coordinates here)
    // Decrement the land counter if the drone is not moving on the z-axis
    this->land_counter_ = (std::abs(velocity_z)  < land_detected_treshold_) ? this->land_counter_ - dt : countdown_to_disarm_;

    // Check if the drone has landed
    return this->land_counter_ <= 0.0 ? true : false;
}

void LandMode::update(double dt) {

    // Get the current state of the drone
    State curr_state = this->get_vehicle_state();

    // Update the target Z position based on the current land speed
    this->target_pos_[2] += this->land_speed_ * dt;

    // Set the controller to track the position which is slighlty bellow the vehicle, but keep the original orientation
    this->set_position(this->target_pos_, this->target_yaw_);

    // Check if the position is no longer changing - if so, it means that the drone has landed and we should signal the mode as finished
    if (check_land_complete(curr_state.velocity[2], dt)) signal_mode_finished();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::LandMode, autopilot::Mode)