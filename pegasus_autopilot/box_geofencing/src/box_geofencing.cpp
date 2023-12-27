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
#include "box_geofencing/box_geofencing.hpp"

namespace autopilot {

void BoxGeofencing::initialize() {

    // Read the geofencing mechanism to be used from the parameter server
    node_->declare_parameter<std::vector<double>>("autopilot.BoxGeofencing.limits_x", {-1.0, 1.0});
    node_->declare_parameter<std::vector<double>>("autopilot.BoxGeofencing.limits_y", {-1.0, 1.0});
    node_->declare_parameter<std::vector<double>>("autopilot.BoxGeofencing.limits_z", {-1.0, 1.0});

    rclcpp::Parameter limits_x = node_->get_parameter("autopilot.BoxGeofencing.limits_x");
    rclcpp::Parameter limits_y = node_->get_parameter("autopilot.BoxGeofencing.limits_y");
    rclcpp::Parameter limits_z = node_->get_parameter("autopilot.BoxGeofencing.limits_z");
    
    // Initialize the limits
    this->limits_x_ = Eigen::Vector2d(limits_x.as_double_array()[0], limits_x.as_double_array()[1]);
    this->limits_y_ = Eigen::Vector2d(limits_y.as_double_array()[0], limits_y.as_double_array()[1]);
    this->limits_z_ = Eigen::Vector2d(limits_z.as_double_array()[0], limits_z.as_double_array()[1]);

    // Check that the limits are valid
    if (this->limits_x_(0) > this->limits_x_(1) || this->limits_y_(0) > this->limits_y_(1) || this->limits_z_(0) > this->limits_z_(1)) {
        RCLCPP_ERROR(node_->get_logger(), "The limits of the box geofencing are not valid. The lower limit must be smaller than the upper limit");
        throw std::runtime_error("The limits of the box geofencing are not valid. The lower limit must be smaller than the upper limit");
    }

    // Log the limits
    RCLCPP_INFO(node_->get_logger(), "The limits of the box geofencing are: x = [%.2f, %.2f]", this->limits_x_(0), this->limits_x_(1));
    RCLCPP_INFO(node_->get_logger(), "The limits of the box geofencing are: y = [%.2f, %.2f]", this->limits_y_(0), this->limits_y_(1));
    RCLCPP_INFO(node_->get_logger(), "The limits of the box geofencing are: z = [%.2f, %.2f]", this->limits_z_(0), this->limits_z_(1));
} 

bool BoxGeofencing::check_geofencing_violation() {

    // Get the current position of the vehicle
    Eigen::Vector3d position = get_vehicle_state_().position;

    // Check if the position is outside the limits
    if(position(0) < limits_x_(0) || position(0) > limits_x_(1) || position(1) < limits_y_(0) || position(1) > limits_y_(1) || position(2) < limits_z_(0) || position(2) > limits_z_(1)) {
        return true;
    }

    return false;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::BoxGeofencing, autopilot::Geofencing)