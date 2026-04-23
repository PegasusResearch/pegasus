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
#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_follow_trajectory.hpp"

namespace autopilot {

FollowTrajectoryMode::~FollowTrajectoryMode() {}

void FollowTrajectoryMode::initialize() {

    this->node_->declare_parameter<double>("autopilot.FollowTrajectoryMode.position_error_threshold", 1.0);
    this->position_error_threshold_ = node_->get_parameter("autopilot.FollowTrajectoryMode.position_error_threshold").as_double();

    RCLCPP_INFO(this->node_->get_logger(), "FollowTrajectoryMode initialized with position_error_threshold: %f", this->position_error_threshold_);
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
    desired_position_ = Eigen::Vector3d::Zero();
    desired_velocity_ = Eigen::Vector3d::Zero();
    desired_acceleration_ = Eigen::Vector3d::Zero();
    desired_jerk_ = Eigen::Vector3d::Zero();
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
    desired_position_ = trajectory_manager_->position(gamma_);
    desired_velocity_ = trajectory_manager_->velocity(gamma_, d_gamma_);
    desired_acceleration_ = trajectory_manager_->acceleration(gamma_, d_gamma_, d2_gamma_);
    desired_jerk_ = trajectory_manager_->jerk(gamma_, d_gamma_, d2_gamma_, d3_gamma_);

    // Get the desired yaw and yaw_rate from the trajectory
    desired_yaw_ = Pegasus::Rotations::rad_to_deg(trajectory_manager_->yaw(gamma_));
    desired_yaw_rate_ = Pegasus::Rotations::rad_to_deg(trajectory_manager_->d_yaw(gamma_));

    // Integrate the virtual target position over time
    integrate_gamma(dt);
}

void FollowTrajectoryMode::integrate_gamma(double dt) {

    // Get the difference between the current position and the desired position
    State curr_state = get_vehicle_state();

    // Compute the position error norm
    double pos_error_norm = (desired_position_ - curr_state.position).norm();

    // When integrating the virtual target, check if it is very far away from the path. If so, wait until the vehicle gets closer to the path before
    // increasing the speed of the virtual target to its nominal speed.
    if (pos_error_norm < this->position_error_threshold_) {
        
        // This is an approximation. Since we are modulating the speed of the virtual target, we would also need to modulate the acceleration and jerk of the virtual target.
        // However, in close to the path, and in the tracking regime, d_gamma_ -> vd, and we can assumte that the acceleration and jerk of the virtual target are not too different from the nominal acceleration and jerk 
        d3_gamma_ = trajectory_manager_->d2_vd(gamma_);
        d2_gamma_ = trajectory_manager_->d_vd(gamma_);

        // Make the speed of the virtual target depend on the position error, to have a smooth transition from an initial position
        // far away from the path and converging to the nominal speed of the trajectory when we get closer to the path.
        d_gamma_ = std::exp(1 + (std::pow(this->position_error_threshold_,2)/(std::pow(pos_error_norm,2) - std::pow(this->position_error_threshold_,2)))) * trajectory_manager_->vd(gamma_);
        
        // Integrate the virtual target position over time
        gamma_ += d_gamma_ * dt;
    
    // If the position error is large
    } else {
        
        // Set the velocity and acceleration of the virtual target to zero
        d_gamma_ = 0.0;
        d2_gamma_ = 0.0;
        d3_gamma_ = 0.0;
    }

    // Saturate gamma to be between 0 and the max value of the trajectory
    gamma_ = std::max(0.0, std::min(gamma_, trajectory_manager_->max_gamma()));

    // Since some speed profiles are "constant speed", we need to check if we have reached 
    // the end of the trajectory and set the speed to zero if we have reached the end of the
    // trajectory to avoid overshooting the final position
    if (gamma_ >= trajectory_manager_->max_gamma()) {
        d_gamma_ = 0.0;
        d2_gamma_ = 0.0;
        d3_gamma_ = 0.0;
    }
}

bool FollowTrajectoryMode::check_finished() {

    // Get the max gamma value for the trajectory
    double max_gamma = trajectory_manager_->max_gamma();

    // Check if the virtual target is not yet at the end of the trajectory
    if(gamma_ < max_gamma) return false;

    // If gamma is already at the end of the trajectory, check if the vehicle is close enough to the final position to consider the trajectory following mission finished
    if(gamma_ >= max_gamma) {

         // Check if the vehicle is close to the final position already
        Eigen::Vector3d position = get_vehicle_state().position;

        // Compute the position error
        Eigen::Vector3d pos_error = desired_position_ - position;

        // If the position error is small, then we can consider the trajectory following mission finished and signal the state machine to change to the next mode
        if (pos_error.norm() < 0.1) {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory Tracking mission finished.");
            signal_mode_finished(); 
            return true;
        }
    }
    return false;
}

bool FollowTrajectoryMode::exit() {
    
    // Reset the parametric values
    gamma_ = 0.0;
    d_gamma_ = 0.0;
    d2_gamma_ = 0.0;
    d3_gamma_ = 0.0;

    // Reset the desired targets
    desired_position_ = Eigen::Vector3d::Zero();
    desired_velocity_ = Eigen::Vector3d::Zero();
    desired_acceleration_ = Eigen::Vector3d::Zero();
    desired_jerk_ = Eigen::Vector3d::Zero();
    desired_yaw_ = 0.0;
    desired_yaw_rate_ = 0.0;

    // Return with success
    return true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)