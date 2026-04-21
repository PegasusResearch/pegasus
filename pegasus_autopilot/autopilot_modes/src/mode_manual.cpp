/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
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
#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_manual.hpp"

namespace autopilot {

ManualMode::~ManualMode() {}

void ManualMode::initialize() {

    // Declare and get the manual command topic parameter
    node_->declare_parameter<std::string>("autopilot.ManualMode.subscribers.manual_command", "manual_command");
    manual_command_topic_ = node_->get_parameter("autopilot.ManualMode.subscribers.manual_command").as_string();
}

bool ManualMode::enter() {

    // Subscribe to the manual command topic
    manual_command_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        manual_command_topic_,
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            // Update the target velocity and yaw rate based on the received manual command
            this->target_vel.x() = msg->twist.linear.x;
            this->target_vel.y() = msg->twist.linear.y;
            this->target_vel.z() = msg->twist.linear.z;
            this->target_yaw_rate = Pegasus::Rotations::rad_to_deg(msg->twist.angular.z);
            this->inertial_frame_command = msg->header.frame_id == "map";  // Check if the command is in the inertial frame

            // Update the last received manual command timestamp
            this->last_manual_command_time_ = msg->header.stamp;
        }
    );

    // Set the target position and velocity to the current position and zero, respectively, to avoid sudden jumps when entering the manual mode
    State curr_state = this->get_vehicle_state();
    
    this->target_pos = curr_state.position;
    this->target_vel = Eigen::Vector3d::Zero();
    
    this->target_yaw = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude));
    this->target_yaw_rate = 0.0;

    // Reset the inertial frame command and the manual time
    this->inertial_frame_command = true;
    this->last_manual_command_time_ = node_->now();

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool ManualMode::exit() {

    // Unsubscribe from the manual command topic by resetting the subscription object
    manual_command_sub_.reset();
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void ManualMode::update(double dt) {

    // Check if we have received a manual command recently, if not, do not update the target position and yaw (i.e. stay where we are)
    auto last_command_age = (node_->now() - last_manual_command_time_).seconds();
    
    // Give a grace period of 1 second after losing the manual command signal, to avoid switching to the failsafe mode due to temporary communication issues or delays in receiving the manual command
    if (last_command_age > 1.0 && last_command_age < 2.0) {
        return;
    // If we haven't received a manual command for a long time, we can consider that the manual control signal has been lost, and we can switch to a failsafe mode (e.g. hover or land)
    } else if (last_command_age >= 2.0) {
        RCLCPP_WARN(node_->get_logger(), "Manual command signal lost! Switching to failsafe mode...");
        signal_mode_finished();  // Signal the state machine that the mode has finished operating, so that it can switch to the failsafe mode
        return;
    }

    // Otherwise, just update the target position and yaw based on the received manual command
    this->update_target_position(dt);
    
    // Set the controller to track the target position and yaw
    this->controller_->set_position(this->target_pos, this->target_vel, this->target_yaw, this->target_yaw_rate, dt);
}

void ManualMode::update_target_position(double dt) {

    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Get the current yaw of the vehicle in radians
    double yaw_rad = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // If the command is in the "body-like" frame, then we need to rotate the target velocity to the inertial frame
    if (!this->inertial_frame_command) {
        
        // Get the rotation matrix from the "body-aligned" frame to the inertial frame (discard roll and pitch, only consider yaw)
        Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
        Rz << cos(yaw_rad), -sin(yaw_rad), 0,
              sin(yaw_rad), cos(yaw_rad), 0,
              0, 0, 1;

        // Rotate the target velocity to the inertial frame
        this->target_vel = Rz * this->target_vel;
    }

    // Given the current state and the target velocity and yaw rate, we can compute the target position and yaw to track
    this->target_pos += this->target_vel * dt;
    this->target_yaw = Pegasus::Rotations::rad_to_deg(yaw_rad) + this->target_yaw_rate * dt;
}


} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::ManualMode, autopilot::Mode)