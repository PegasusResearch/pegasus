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
#include "autopilot_controllers/onboard_controller.hpp"

namespace autopilot {

OnboardController::~OnboardController() {}

void OnboardController::initialize() {

    // Initialize the ROS 2 subscribers to the control topics
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_position", "control_position");
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_body_velocity", "control_body_velocity");
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_inertial_velocity", "control_inertial_velocity");
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_inertial_acceleration", "control_inertial_acceleration");
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_attitude", "control_attitude");
    node_->declare_parameter<std::string>("autopilot.OnboardController.publishers.control_attitude_rate", "control_attitude_rate");

    // Create the publishers
    position_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlPosition>(node_->get_parameter("autopilot.OnboardController.publishers.control_position").as_string(), rclcpp::SensorDataQoS());
    body_velocity_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlVelocity>(node_->get_parameter("autopilot.OnboardController.publishers.control_body_velocity").as_string(), rclcpp::SensorDataQoS());
    inertial_velocity_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlVelocity>(node_->get_parameter("autopilot.OnboardController.publishers.control_inertial_velocity").as_string(), rclcpp::SensorDataQoS());
    inertial_acceleration_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAcceleration>(node_->get_parameter("autopilot.OnboardController.publishers.control_inertial_acceleration").as_string(), rclcpp::SensorDataQoS());
    attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.OnboardController.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.OnboardController.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());

    // Log that the OnboardController was initialized
    RCLCPP_INFO(node_->get_logger(), "OnboardController initialized");
}

void OnboardController::set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate, double dt) {

    // Ignore the velocity, acceleration, jerk, snap and yaw_rate references
    (void) velocity;
    (void) acceleration;
    (void) jerk;
    (void) snap;
    (void) yaw_rate;
    (void) dt;

    // Set the position control message
    position_msg_.position[0] = position[0];
    position_msg_.position[1] = position[1];
    position_msg_.position[2] = position[2];
    position_msg_.yaw = yaw;

    // Publish the position control message for the controller to track
    position_publisher_->publish(position_msg_);
}

void OnboardController::set_body_velocity(const Eigen::Vector3d& velocity, double yaw_rate, double dt) {

    // Ignore dt
    (void) dt;

    // Set the desired body velocity
    body_velocity_msg_.velocity[0] = velocity[0];
    body_velocity_msg_.velocity[1] = velocity[1];
    body_velocity_msg_.velocity[2] = velocity[2];
    body_velocity_msg_.yaw = yaw_rate;

    // Publish the body velocity control message for the controller to track
    body_velocity_publisher_->publish(body_velocity_msg_);
}

void OnboardController::set_inertial_velocity(const Eigen::Vector3d& velocity, double yaw, double dt) {

    // Ignore dt
    (void) dt;

    // Set the inertial velocity control message
    inertial_velocity_msg_.velocity[0] = velocity[0];
    inertial_velocity_msg_.velocity[1] = velocity[1];
    inertial_velocity_msg_.velocity[2] = velocity[2];
    inertial_velocity_msg_.yaw = yaw;

    // Publish the inertial velocity control message for the controller to track
    inertial_velocity_publisher_->publish(inertial_velocity_msg_);
}

void OnboardController::set_inertial_acceleration(const Eigen::Vector3d& acceleration, double dt) {

    // Ignore dt
    (void) dt;

    // Set the inertial acceleration control message
    inertial_acceleration_msg_.acceleration[0] = acceleration[0];
    inertial_acceleration_msg_.acceleration[1] = acceleration[1];
    inertial_acceleration_msg_.acceleration[2] = acceleration[2];

    // Publish the inertial acceleration control message for the controller to track
    inertial_acceleration_publisher_->publish(inertial_acceleration_msg_);
}

void OnboardController::set_attitude(const Eigen::Vector3d & attitude, double thrust_force, double dt) {

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

void OnboardController::set_attitude_rate(const Eigen::Vector3d & attitude_rate, double thrust_force, double dt) {

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

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::OnboardController, autopilot::Controller)