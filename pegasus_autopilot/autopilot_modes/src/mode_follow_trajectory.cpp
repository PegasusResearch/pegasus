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
#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_follow_trajectory.hpp"

namespace autopilot {

FollowTrajectoryMode::~FollowTrajectoryMode() {}

void FollowTrajectoryMode::initialize() {
    RCLCPP_INFO(this->node_->get_logger(), "FollowTrajectoryMode initialized");
}

bool FollowTrajectoryMode::enter() {

    // Check if the path is empty. If it is, return with error
    if(trajectory_manager_->empty()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Path is empty. Cannot follow an empty path.");
        return false;
    }

    // Reset the tracking references
    gamma_ = 0.0;
    d_gamma_ = 0.0;
    d2_gamma_ = 0.0;
    d3_gamma_ = 0.0;

    // Reset the desired targets
    desired_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_jerk_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_yaw_ = 0.0;
    desired_yaw_rate_ = 0.0;

    // Otherwise, enter the trajectory following mode
    return true;
}

void FollowTrajectoryMode::update(double dt) {

    // Update the current reference on the path to follow
    update_reference(dt);

    // Call the controller
    controller_->set_position(desired_position_, desired_velocity_, desired_acceleration_, desired_jerk_, desired_yaw_, desired_yaw_rate_, dt);

    // Check if we have reached the end of the path
    check_finished();
}

void FollowTrajectoryMode::update_reference(double dt) {

    // Check if the trajectory is empty.
    if(trajectory_manager_->empty()) {

        // Get the vehicle state
        State curr_state = get_vehicle_state();

        // Set the desired position to the current position
        desired_position_ = curr_state.position;
        desired_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        desired_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        desired_jerk_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Set the desired yaw and yaw rate to the current yaw and yaw rate
        desired_yaw_ = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude));
        desired_yaw_rate_ = 0.0;

        // Signal the mode is finished
        RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory is empty. Changing operation mode.");
        signal_mode_finished();
        return;
    }

    // Update the desired position, velocity, acceleration and jerk from the trajectory
    desired_position_ = trajectory_manager_->pd(gamma_);
    desired_velocity_ = trajectory_manager_->d_pd(gamma_) * d_gamma_;
    desired_acceleration_ = (trajectory_manager_->d2_pd(gamma_) * std::pow(d_gamma_, 2)) + (trajectory_manager_->d_pd(gamma_) * std::pow(d2_gamma_, 2));
    desired_jerk_ = (trajectory_manager_->d3_pd(gamma_) * std::pow(d_gamma_, 3))
        + (2 * trajectory_manager_->d2_pd(gamma_) * d_gamma_ * d2_gamma_)
        + (trajectory_manager_->d2_pd(gamma_) * d2_gamma_)
        + (trajectory_manager_->d_pd(gamma_) * d3_gamma_);

    // Get the desired yaw and yaw_rate from the trajectory
    desired_yaw_ = Pegasus::Rotations::rad_to_deg(trajectory_manager_->yaw(gamma_));
    desired_yaw_rate_ = Pegasus::Rotations::rad_to_deg(trajectory_manager_->d_yaw(gamma_));

    // Integrate the virtual target position over time
    d3_gamma_ = 0.0;
    d2_gamma_ = trajectory_manager_->d_vd(gamma_);
    d_gamma_ = trajectory_manager_->vd(gamma_);
    gamma_ += d_gamma_ * dt;
}

bool FollowTrajectoryMode::check_finished() {

    // Check if the virtual target is already at the end of the trajectory
    if(gamma_ < trajectory_manager_->max_gamma()) return false;

    // Check if the vehicle is close to the final position already
    Eigen::Vector3d position = get_vehicle_state().position;

    // Compute the position error
    Eigen::Vector3d pos_error = desired_position_ - position;

    // If the position error is too big, return false
    if (pos_error.norm() > 0.1) return false;

    // Otherwise, return true
    RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory Tracking mission finished.");
    signal_mode_finished();
    return true;
}

bool FollowTrajectoryMode::exit() {
    
    // Reset the parametric values
    gamma_ = 0.0;
    d_gamma_ = 0.0;
    d2_gamma_ = 0.0;
    d3_gamma_ = 0.0;

    // Reset the desired targets
    desired_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_jerk_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_yaw_ = 0.0;
    desired_yaw_rate_ = 0.0;

    // Return with success
    return true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)