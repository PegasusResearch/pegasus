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

    // Otherwise, enter the trajectory following mode
    return true;
}

bool FollowTrajectoryMode::exit() {
    
    // Reset the parametric values
    gamma_ = 0.0;
    d_gamma_ = 0.0;
    dd_gamma_ = 0.0;

    // Reset the desired targets
    desired_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_acceleration_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Return with success
    return true;
}

void FollowTrajectoryMode::update_reference(double dt) {

    // Get the desired speed progression of the path at the current location
    d_gamma_ = trajectory_manager_->vd(gamma_).value();
    dd_gamma_ = trajectory_manager_->d_vd(gamma_).value();

    // Update the desired position, velocity and acceleration from the path
    desired_position_ = trajectory_manager_->pd(gamma_).value();
    desired_velocity_ = trajectory_manager_->d_pd(gamma_).value() * d_gamma_;
    desired_acceleration_ = (trajectory_manager_->d2_pd(gamma_).value() * std::pow(d_gamma_, 2)) + (trajectory_manager_->d_pd(gamma_).value() * std::pow(dd_gamma_, 2));

    // Update the desired yaw from the tangent to the path
    // TODO: make the this more general later on
    desired_yaw_ = 0.0; //path_.tangent_angle(gamma_).value();

    // Integrate the parametric value (virtual target) over time
    gamma_ += d_gamma_ * dt;

    // Saturate the parametric value
    gamma_ = std::min(std::max(trajectory_manager_->min_gamma(), gamma_), trajectory_manager_->max_gamma());
}

bool FollowTrajectoryMode::check_finished() {

    // Get the stats from the PID controllers
    // Eigen::Vector3d pos_error(
    //     controllers_[0]->get_statistics().error_p, 
    //     controllers_[1]->get_statistics().error_p, 
    //     controllers_[2]->get_statistics().error_p);

    // Compute the norm of the position error
    // double pos_error_norm = pos_error.norm();

    // // Check if the path is finished
    // if(gamma_ >= trajectory_manager_->max_gamma() &&  pos_error_norm < 0.1) {
    //     RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory Tracking mission finished.");
    //     signal_mode_finished();
    //     return true;
    // }

    return false;
}

void FollowTrajectoryMode::update(double dt) {

    // Update the current reference on the path to follow
    update_reference(dt);

    // Check if we have reached the end of the path
    check_finished();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)