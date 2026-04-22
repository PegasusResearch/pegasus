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
#include "autopilot_modes/mode_smooth_waypoint.hpp"

namespace autopilot {

SmoothWaypointMode::~SmoothWaypointMode() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

void SmoothWaypointMode::initialize() {

    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.SmoothWaypointMode.set_waypoint_service", "set_smooth_waypoint"); 
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.target_speed", 1.5);
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.speed_profile_k", 1.0); // This value should be >= 1 to have a speed profile that starts and ends at zero, and has a maximum in the middle of the trajectory

    // Set the class members from parameters (avoid local shadowing).
    this->target_speed_ = node_->get_parameter("autopilot.SmoothWaypointMode.target_speed").as_double();
    this->k_ = node_->get_parameter("autopilot.SmoothWaypointMode.speed_profile_k").as_double();

    // Create the service server for setting the waypoint
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(node_->get_parameter("autopilot.SmoothWaypointMode.set_waypoint_service").as_string(), std::bind(&SmoothWaypointMode::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));    
    
    // Log the initialization of the mode
    RCLCPP_INFO(this->node_->get_logger(), "SmoothWaypointMode initialized with target speed %f m/s, speed profile gain %f, and service server '%s'", this->target_speed_, this->k_, node_->get_parameter("autopilot.SmoothWaypointMode.set_waypoint_service").as_string().c_str());
}

bool SmoothWaypointMode::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->waypoint_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Waypoint not set - cannot enter SmoothWaypoint mode.");
        return false;
    }

    // Set the trajectory to be followed by the controller (to start at the current position)
    this->set_trajectory_parameters();
    
    // Reset the waypoint flag (to make sure we do not enter twice in this mode without setting a new waypoint)
    this->waypoint_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

void SmoothWaypointMode::set_trajectory_parameters() {

    // Get the current position to use as the start position for the trajectory
    this->start_pos_ = this->get_vehicle_state().position;
    this->trajectory_slope_ = (this->target_pos_ - this->start_pos_);

    // this->start_yaw_ = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(this->get_vehicle_state().attitude));

    // Compute how the max target speed translates to the max gamma_dot value
    double trajectory_length = this->trajectory_slope_.norm();

    // Check if we are already at the target position - if so, set gamma_dot_max to zero to avoid numerical issues with the speed profile
    if (trajectory_length > 0.01) {
        this->gamma_dot_max_ = this->target_speed_ / trajectory_length;
    } else {
        RCLCPP_WARN(this->node_->get_logger(), "Start and target positions are the same - setting gamma_dot_max to zero.");
        this->gamma_dot_max_ = 0.0;
    }

    // Initialize gamma with a very small gamma value to avoid numerical issues with the speed profile at gamma = 0
    this->gamma_ = 1/trajectory_length * 1E-4;

    // LOG the trajectory parameters
    RCLCPP_INFO(this->node_->get_logger(), "Trajectory slope: (%f, %f, %f), trajectory length: %f, gamma_dot_max: %f", this->trajectory_slope_[0], this->trajectory_slope_[1], this->trajectory_slope_[2], trajectory_length, this->gamma_dot_max_);
}   

bool SmoothWaypointMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void SmoothWaypointMode::update(double dt) {

    // Integrate the virtual target along the path line 
    gamma_ += (gamma_dot(gamma_)*dt) + (0.5*gamma_ddot(gamma_)*std::pow(dt,2));

    // Saturate gamma to be between 0 and 1
    gamma_ = std::min(std::max(0.0, gamma_), 1.0);

    RCLCPP_INFO(this->node_->get_logger(), "Gamma: %f, Gamma_dot: %f, Gamma_ddot: %f", gamma_, gamma_dot(gamma_), gamma_ddot(gamma_));

    // Get the desired position, velocity, acceleration and jerk at the current gamma value
    Eigen::Vector3d pos = desired_position(gamma_);
    Eigen::Vector3d vel = desired_velocity(gamma_);
    Eigen::Vector3d acc = desired_acceleration(gamma_);

    // Set the controller to track the target position and attitude
    this->controller_->set_position(pos, vel, acc, this->target_yaw_, 0.0, dt);
}

void SmoothWaypointMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->target_pos_[0] = request->position[0];
    this->target_pos_[1] = request->position[1];
    this->target_pos_[2] = request->position[2];
    this->target_yaw_ = request->yaw;

    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f) with yaw %f", this->target_pos_[0], this->target_pos_[1], this->target_pos_[2], this->target_yaw_);

    // Set the trajectory to be followed by the controller (to start at the current position)
    this->set_trajectory_parameters();
}

double SmoothWaypointMode::gamma_dot(double gamma) const {
    // Define a speed profile that starts and ends at zero, and has a maximum in the middle of the trajectory
    return gamma_dot_max_ * std::pow(std::sin(M_PI * gamma), k_);
}

double SmoothWaypointMode::gamma_ddot(double gamma) const {
    return gamma_dot_max_*k_*M_PI*std::cos(M_PI*gamma)*std::pow(std::sin(M_PI*gamma), k_-1);
}

double SmoothWaypointMode::gamma_dddot(double gamma) const {
    return -std::pow(M_PI,2)*gamma_dot_max_*k_*std::pow(std::sin(M_PI*gamma),k_-2)*(std::pow(std::sin(M_PI*gamma),2) + (1-k_)*std::pow(std::cos(M_PI*gamma),2));
}

Eigen::Vector3d SmoothWaypointMode::desired_position(double gamma) const {
    return start_pos_ + gamma*trajectory_slope_;
}

Eigen::Vector3d SmoothWaypointMode::desired_velocity(double gamma) const {
    return gamma_dot(gamma) * trajectory_slope_;
}

Eigen::Vector3d SmoothWaypointMode::desired_acceleration(double gamma) const {
    return gamma_ddot(gamma) * trajectory_slope_;
}

Eigen::Vector3d SmoothWaypointMode::desired_jerk(double gamma) const {
    return gamma_dddot(gamma) * trajectory_slope_;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::SmoothWaypointMode, autopilot::Mode)