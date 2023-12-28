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
#include "static_trajectories/lemniscate.hpp"

namespace autopilot {

Lemniscate::Lemniscate(const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const double radius, const double vehicle_speed) : 
    StaticTrajectory(0.0, 1.0), vehicle_speed_(vehicle_speed), center_(center), normal_(normal), radius_(radius) {

    // ------------------------
    // Initialize the rotation matrix with the rotation 
    // information encoded in the normal vector
    // -----------------------
    
    // Step 1 - Check that the normal vector != [0, 0, 1] and [0,0, -1] otherwize we would not need to rotate
    // and the rotation matrix would be hill-posed using the following method
    Eigen::Vector3d base_normal(0.0, 0.0, 1.0);
    Eigen::Vector3d absolute_normal = normal_.array().abs();

    // Because we are using doubles, we must check if the vectors are not approximatelly [0, 0, 1] 
    if ((absolute_normal - base_normal).norm() > 0.0001) {

        // Step 2 - Now that we know that the vector is valid, then compute the rotation matrix
        Eigen::Vector3d u3 = normal_.normalized();
        Eigen::Vector3d u1 = (u3.cross(base_normal)).normalized();
        Eigen::Vector3d u2 = (u3.cross(u1)).normalized();

        // Step 3 - assign the normalized vectors to the columns of the rotation matrix
        for(int i = 0; i < 3; i++) rotation_(0,i) = u1(i);
        for(int i = 0; i < 3; i++) rotation_(1,i) = u2(i);
        for(int i = 0; i < 3; i++) rotation_(2,i) = u3(i);
    }
}

Eigen::Vector3d Lemniscate::pd(const double gamma) const {
    
    // Compute the location of the 2D circle in a plane centered around [x,y, 0.0]
    Eigen::Vector3d pd;
    double denominator = 1 + std::pow(sin(gamma * 2 * M_PI), 2);
    pd[0] = radius_ * cos(gamma * 2 * M_PI) / denominator;
    pd[1] = radius_ * sin(gamma * 2 * M_PI) * cos(gamma * 2 * M_PI) / denominator;
    pd[2] = 0.0;
    
    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    pd = rotation_ * pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return pd + center_;
}

Eigen::Vector3d Lemniscate::d_pd(const double gamma) const {

    // Compute the first derivative of the circle with respect to the parametric value
    Eigen::Vector3d d_pd;
    d_pd[0] = -(2 * M_PI * radius_ * sin(2 * M_PI * gamma) * (std::pow(sin(2 * M_PI * gamma),2) + (2 * std::pow(2 * M_PI * gamma, 2)) + 1)) / std::pow(std::pow(sin(2 * M_PI * gamma), 2) + 1, 2);
    d_pd[1] = -(2 * M_PI * radius_ * (std::pow(sin(2 * M_PI * gamma), 4) + (std::pow(cos(2 * M_PI * gamma), 2) + 1) * std::pow(sin(2 * M_PI * gamma), 2) - std::pow(cos(2 * M_PI * gamma), 2))) / std::pow(std::pow(sin(2 * M_PI * gamma), 2) + 1, 2);
    d_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    return rotation_ * d_pd;
}

Eigen::Vector3d Lemniscate::d2_pd(const double gamma) const {

    // Compute the second derivative of the circle with respect to the parametric value
    Eigen::Vector3d dd_pd;
    dd_pd[0] = (4 * std::pow(M_PI, 2) * radius_ * cos(2 * M_PI * gamma) * (5 * std::pow(sin(2 * M_PI * gamma), 4) + (6*std::pow(cos(2 * M_PI * gamma), 2) + 4) * std::pow(sin(2 * M_PI * gamma), 2) - 2 * std::pow(cos(2 * M_PI * gamma), 2) - 1)) / std::pow(std::pow(sin(2 * M_PI * gamma), 2) + 1, 3);
    dd_pd[1] = (8 * std::pow(M_PI, 2) * radius_ * cos(2 * M_PI * gamma) * sin(2 * M_PI * gamma) * (std::pow(sin(2 * M_PI * gamma), 4) + (std::pow(cos(2 * M_PI * gamma), 2) - 1) * std::pow(sin(2 * M_PI * gamma), 2) - 3 * std::pow(cos(2 * M_PI * gamma), 2) - 2)) / std::pow(std::pow(sin(2 * M_PI * gamma), 2) + 1, 3);
    dd_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    return rotation_ * dd_pd;
}

double Lemniscate::vehicle_speed(const double gamma) const {
    return vehicle_speed_;
}

double Lemniscate::vd(const double gamma) const {

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

void LemniscateFactory::initialize() {
    
    // Load the service topic from the parameter server
    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.LemniscateFactory.service", "path/add_lemniscate");

    // Advertise the service to add a line to the path
    add_lemniscate_service_ = node_->create_service<pegasus_msgs::srv::AddLemniscate>(node_->get_parameter("autopilot.StaticTrajectoryManager.LemniscateFactory.service").as_string(), std::bind(&LemniscateFactory::lemniscate_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void LemniscateFactory::lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response) {
    
    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding lemniscate to path. Speed: " << request->speed.parameters[0] << ", center: [" << request->center[0] << "," << request->center[1] << "," << request->center[2] << "], normal: [" << request->normal[0] << "," << request->normal[1] << "," << request->normal[2] << "], radius: " << request->radius << ".");
    
    // Create a new lemniscate
    Lemniscate::SharedPtr lemniscate = std::make_shared<Lemniscate>(
        Eigen::Vector3d(request->center.data()), 
        Eigen::Vector3d(request->normal.data()), 
        request->radius,
        request->speed.parameters[0]);

    // Add the new lemniscate to the path
    this->add_trajectory_to_manager(lemniscate);

    // Update the response
    response->success = true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::LemniscateFactory, autopilot::StaticTrajectoryFactory)