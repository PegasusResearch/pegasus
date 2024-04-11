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
#include "autopilot_modes/mode_hold.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

HoldMode::~HoldMode() {}

void HoldMode::initialize() {
    
    // Log that the mode has been initialized successfully
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

    // Set the target yaw to the current yaw of the drone (in degrees)
    this->target_yaw_ = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude));

    // Log the Hold position
    RCLCPP_WARN(this->node_->get_logger(), "Hold position: [%f, %f, %f] and yaw: %f", this->target_pos[0], this->target_pos[1], this->target_pos[2], this->target_yaw_);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool HoldMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void HoldMode::update(double dt) {

    // Set the controller to track the target position and attitude
    this->controller_->set_position(this->target_pos, this->target_yaw_, dt);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::HoldMode, autopilot::Mode)