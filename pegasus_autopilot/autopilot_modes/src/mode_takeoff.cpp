/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this 
 * software must display the following acknowledgement: This product 
 * includes software developed by Project Pegasus.
 * 4. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived 
 * from this software without specific prior written permission.
 *
 * Additional Restrictions:
 * 4. The Software shall be used for non-commercial purposes only. 
 * This includes, but is not limited to, academic research, personal 
 * projects, and non-profit organizations. Any commercial use of the 
 * Software is strictly prohibited without prior written permission 
 * from the copyright holders.
 * 5. The Software shall not be used, directly or indirectly, for 
 * military purposes, including but not limited to the development 
 * of weapons, military simulations, or any other military applications. 
 * Any military use of the Software is strictly prohibited without 
 * prior written permission from the copyright holders.
 * 6. The Software may be utilized for academic research purposes, 
 * with the condition that proper acknowledgment is given in all 
 * corresponding publications.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#include "autopilot_modes/mode_takeoff.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

TakeoffMode::~TakeoffMode() {}

void TakeoffMode::initialize() {

    // Initialize the default target altitude
    node_->declare_parameter<float>("autopilot.TakeoffMode.takeoff_altitude", -1.0f);
    target_altitude = node_->get_parameter("autopilot.TakeoffMode.takeoff_altitude").as_double();

    // Log the default takeoff altitude
    RCLCPP_INFO(this->node_->get_logger(), "Takeoff altitude set to %.2f m.", this->target_altitude);

    // Initialize the service server for setting the takeoff altitude
    node_->declare_parameter<std::string>("autopilot.TakeoffMode.set_takeoff_altitude_service", "set_takeoff_altitude");
    altitude_service_ = node_->create_service<pegasus_msgs::srv::Takeoff>(node_->get_parameter("autopilot.TakeoffMode.set_takeoff_altitude_service").as_string(), std::bind(&TakeoffMode::altitude_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Log that the takeoff mode has been initialized successfully
    RCLCPP_INFO(this->node_->get_logger(), "TakeoffMode initialized");
}

bool TakeoffMode::enter() {
    
    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->takeoff_pos[0] = curr_state.position[0];
    this->takeoff_pos[1] = curr_state.position[1];
    this->takeoff_pos[2] = curr_state.position[2] + this->target_altitude;

    // Set the target yaw to the current yaw of the drone (in degrees)
    this->takeoff_yaw = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude));

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool TakeoffMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void TakeoffMode::update(double dt) {

    // Set the controller to track the target position and attitude
    this->controller_->set_position(this->takeoff_pos, this->takeoff_yaw, dt);
}

void TakeoffMode::altitude_callback(const pegasus_msgs::srv::Takeoff::Request::SharedPtr request, const pegasus_msgs::srv::Takeoff::Response::SharedPtr response) {

    // Set the target altitude
    this->target_altitude = request->height;
    response->success = pegasus_msgs::srv::Takeoff::Response::TRUE;

    RCLCPP_WARN(this->node_->get_logger(), "Takeoff altitude set to %.2f", this->target_altitude);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::TakeoffMode, autopilot::Mode)