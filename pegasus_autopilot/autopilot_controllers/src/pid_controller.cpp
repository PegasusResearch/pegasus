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
#include "autopilot_controllers/pid_controller.hpp"

#include <pegasus_utils/rotations.hpp>
#include <thrust_curves/acceleration_to_attitude.hpp>

namespace autopilot {

PIDController::~PIDController() {}

void PIDController::initialize() {

    // Load the controller gains from the parameter server
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.kp", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.kd", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.ki", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.kff", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.PIDController.gains.max_output", std::vector<double>());

    auto kp = node_->get_parameter("autopilot.PIDController.gains.kp").as_double_array();
    auto kd = node_->get_parameter("autopilot.PIDController.gains.kd").as_double_array();
    auto ki = node_->get_parameter("autopilot.PIDController.gains.ki").as_double_array();
    auto kff = node_->get_parameter("autopilot.PIDController.gains.kff").as_double_array();
    auto min_output = node_->get_parameter("autopilot.PIDController.gains.min_output").as_double_array();
    auto max_output = node_->get_parameter("autopilot.PIDController.gains.max_output").as_double_array();

    // Safety check on the gains (make sure they are there)
    if(kp.size() != 3 || kd.size() != 3 || ki.size() != 3 || kff.size() != 3 || min_output.size() != 3 || max_output.size() != 3) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not read PID position controller gains correctly.");
        throw std::runtime_error("Gains vector was empty");
    }

    // Log the PID gains
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID gains: kp = [" << kp[0] << ", " << kp[1] << ", " << kp[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID gains: kd = [" << kd[0] << ", " << kd[1] << ", " << kd[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID gains: ki = [" << ki[0] << ", " << ki[1] << ", " << ki[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID gains: kff = [" << kff[0] << ", " << kff[1] << ", " << kff[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID min_output = [" << min_output[0] << ", " << min_output[1] << ", " << min_output[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID max_output = [" << max_output[0] << ", " << max_output[1] << ", " << max_output[2] << "]");

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    mass_ = vehicle_constansts.mass;

    // Initialize the ROS 2 subscribers to the control topics
    node_->declare_parameter<std::string>("autopilot.PIDController.publishers.control_attitude", "control_attitude");
    node_->declare_parameter<std::string>("autopilot.PIDController.publishers.control_attitude_rate", "control_attitude_rate");
    node_->declare_parameter<std::string>("autopilot.PIDController.pid_debug_topic", "statistics/pid");

    // Create the publishers
    attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.PIDController.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.PIDController.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());
    statistics_pub_ = node_->create_publisher<pegasus_msgs::msg::PidStatistics>(node_->get_parameter("autopilot.PIDController.pid_debug_topic").as_string(), 1);

    // Log that the PIDController was initialized
    RCLCPP_INFO(node_->get_logger(), "PIDController initialized");
}

void PIDController::set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate, double dt) {

    // Ignore jerk, snap and yaw_rate references
    (void) jerk;
    (void) snap;
    (void) yaw_rate;

    // Get the current state of the vehicle
    State state = get_vehicle_state();
    
    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = position - state.position;
    Eigen::Vector3d vel_error = velocity - state.velocity;

    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    const Eigen::Vector3d g(0.0, 0.0, 9.81);
    for(unsigned int i=0; i < 3; i++) u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], (acceleration[i] * mass_) - g[i], dt);
    
    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, yaw);

    // Set the control output
    Eigen::Vector3d attitude_target = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_thrust[0]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[1]), 
        Pegasus::Rotations::rad_to_deg(attitude_thrust[2]));

    // Send the attitude and thrust to the attitude controller
    set_attitude(attitude_target, attitude_thrust[3]);

    // Update and publish the PID statistics
    update_statistics(position);
    statistics_pub_->publish(pid_statistics_msg_);
}

void PIDController::update_statistics(const Eigen::Vector3d & position_ref) {
    
    // For each PID control [x, y, z]
    for(unsigned int i = 0; i < 3; i++) {

        // Get the statistics from the controller object
        Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

        pid_statistics_msg_.statistics[i].dt = stats.dt;
        pid_statistics_msg_.statistics[i].reference = position_ref[i];
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

void PIDController::set_attitude(const Eigen::Vector3d & attitude, double thrust_force, double dt) {

    // Ignore dt
    (void) dt;

    // Set the attitude control message
    attitude_msg_.attitude[0] = attitude[0];
    attitude_msg_.attitude[1] = attitude[1];
    attitude_msg_.attitude[2] = attitude[2];
    attitude_msg_.thrust = thrust_force;

    // Publish the attitude control message for the controller to track
    attitude_publisher_->publish(attitude_msg_);
}

void PIDController::set_attitude_rate(const Eigen::Vector3d & attitude_rate, double thrust_force, double dt) {

    // Ignore dt
    (void) dt;

    // Set the attitude rate control message
    attitude_rate_msg_.attitude[0] = attitude_rate[0];
    attitude_rate_msg_.attitude[1] = attitude_rate[1];
    attitude_rate_msg_.attitude[2] = attitude_rate[2];
    attitude_rate_msg_.thrust = thrust_force;

    // Publish the attitude rate control message for the controller to track
    attitude_rate_publisher_->publish(attitude_rate_msg_);
}

void PIDController::reset_controller() {
    // Reset the controllers
    for(unsigned int i=0; i < 3; i++) controllers_[i]->reset_controller();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::PIDController, autopilot::Controller)