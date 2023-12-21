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
#include "thrust_curves/acceleration_to_attitude.hpp"

namespace autopilot {

FollowTrajectoryMode::~FollowTrajectoryMode() {}

void FollowTrajectoryMode::initialize() {
    
    // Load the controller gains from the parameter server
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kp", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kd", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.ki", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.kff", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.FollowTrajectoryMode.gains.max_output", std::vector<double>());

    auto kp = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kp").as_double_array();
    auto kd = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kd").as_double_array();
    auto ki = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.ki").as_double_array();
    auto kff = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.kff").as_double_array();
    auto min_output = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.min_output").as_double_array();
    auto max_output = node_->get_parameter("autopilot.FollowTrajectoryMode.gains.max_output").as_double_array();

    // Safety check on the gains (make sure they are there)
    if(kp.size() != 3 || kd.size() != 3 || ki.size() != 3 || kff.size() != 3 || min_output.size() != 3 || max_output.size() != 3) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not read PID position controller gains correctly.");
        throw std::runtime_error("Gains vector was empty");
    }

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    mass_ = vehicle_constansts.mass;

    // ----------------------- Logging Information -------------------------
    node_->declare_parameter<std::string>("autopilot.FollowTrajectoryMode.pid_debug_topic", "statistics/pid");

    // Initialize the publishers for the statistics of the PID controller
    statistics_pub_ = node_->create_publisher<pegasus_msgs::msg::PidStatistics>(node_->get_parameter("autopilot.FollowTrajectoryMode.pid_debug_topic").as_string(), 1);

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

    // Reset the controllers
    for(unsigned int i=0; i < 3; i++) controllers_[i]->reset_controller();

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
    Eigen::Vector3d pos_error(
        controllers_[0]->get_statistics().error_p, 
        controllers_[1]->get_statistics().error_p, 
        controllers_[2]->get_statistics().error_p);

    // Compute the norm of the position error
    double pos_error_norm = pos_error.norm();

    // Check if the path is finished
    if(gamma_ >= trajectory_manager_->max_gamma() &&  pos_error_norm < 0.1) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory Tracking mission finished.");
        signal_mode_finished();
        return true;
    }

    return false;
}

void FollowTrajectoryMode::update_statistics() {
    
    // For each PID control [x, y, z]
    for(unsigned int i = 0; i < 3; i++) {

        // Get the statistics from the controller object
        Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

        pid_statistics_msg_.statistics[i].dt = stats.dt;
        pid_statistics_msg_.statistics[i].reference = desired_position_[i];
        // Fill the feedback errors
        pid_statistics_msg_.statistics[i].error_p = stats.error_p;
        pid_statistics_msg_.statistics[i].error_d = stats.error_d;
        pid_statistics_msg_.statistics[i].integral = stats.integral;
        pid_statistics_msg_.statistics[i].ff_ref = stats.ff_ref;

        // Fill the errors scaled by the gains
        pid_statistics_msg_.statistics[i].p_term = stats.p_term;
        pid_statistics_msg_.statistics[i].d_term = stats.d_term;
        pid_statistics_msg_.statistics[i].i_term = stats.i_term;
        pid_statistics_msg_.statistics[i].ff_term = stats.ff_term;

        // Fill the outputs of the controller
        pid_statistics_msg_.statistics[i].anti_windup_discharge = stats.anti_windup_discharge;
        pid_statistics_msg_.statistics[i].output_pre_sat = stats.output_pre_sat;
        pid_statistics_msg_.statistics[i].output = stats.output;
    }
}

void FollowTrajectoryMode::update(double dt) {

    // Update the current reference on the path to follow
    update_reference(dt);

    // Get the current state of the vehicle
    State state = get_vehicle_state();
    
    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = desired_position_ - state.position;
    Eigen::Vector3d vel_error = desired_velocity_ - state.velocity;
    Eigen::Vector3d accel = desired_acceleration_;

    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    const Eigen::Vector3d g(0.0, 0.0, 9.81);
    for(unsigned int i=0; i < 3; i++) u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], (accel[i] * mass_) - g[i], dt);
    
    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, desired_yaw_);

    // Set the control output
    Eigen::Vector3d attitude_target = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_thrust[0]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[1]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[2]));
    this->controller_->set_attitude(attitude_target, attitude_thrust[3]);

    // Update and publish the PID statistics
    update_statistics();
    statistics_pub_->publish(pid_statistics_msg_);

    // Check if we have reached the end of the path
    check_finished();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::FollowTrajectoryMode, autopilot::Mode)