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
#include "static_trajectories/line.hpp"

namespace autopilot {

Line::Line(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double vehicle_speed) : 
    StaticTrajectory(0.0, 1.0), start_(start), end_(end), vehicle_speed_(vehicle_speed) {    
    slope_ = end_ - start_;
}

Eigen::Vector3d Line::pd(const double gamma) const {
    return start_ + gamma * slope_;
}

Eigen::Vector3d Line::d_pd(const double gamma) const {
    return slope_;
}

double Line::vehicle_speed(const double gamma) const {
    return vehicle_speed_;
}

double Line::vd(const double gamma) const {
    
    // Define a zero velocity variable
    double vd = 0.0;

    // Compute the derivative norm
    double derivative_norm = d_pd(gamma).norm();

    // Convert the speed from the vehicle frame to the path frame
    if(derivative_norm != 0) vd = vehicle_speed_ / derivative_norm;

    // If the speed exploded because the derivative norm was hill posed, then set it to a very small value as something wrong has happened
    if(!std::isfinite(vd)) vd = 0.00000001;

    return vd;
}

void LineFactory::initialize() {

    // Load the service topic from the parameter server
    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.LineFactory.service", "path/add_line");

    // Advertise the service to add a line to the path
    add_line_service_ = node_->create_service<pegasus_msgs::srv::AddLine>(node_->get_parameter("autopilot.StaticTrajectoryManager.LineFactory.service").as_string(), std::bind(&LineFactory::line_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void LineFactory::line_callback(const pegasus_msgs::srv::AddLine::Request::SharedPtr request, const pegasus_msgs::srv::AddLine::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "Adding line to the trajectory. Speed: " << request->speed.parameters[0] << ", start: [" << request->start[0] << "," << request->start[1] << "," << request->start[2] << "], end: [" << request->end[0] << "," << request->end[1] << "," << request->end[2] << "].");

    // Create a new line
    Line::SharedPtr line = std::make_shared<Line>(Eigen::Vector3d(
        request->start.data()), 
        Eigen::Vector3d(request->end.data()), 
        request->speed.parameters[0]);

    // Add the line to the path
    this->add_trajectory_to_manager(line);

    // Set the response to true
    response->success = true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::LineFactory, autopilot::StaticTrajectoryFactory)