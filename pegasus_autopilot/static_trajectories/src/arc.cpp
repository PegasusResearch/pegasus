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
#include "static_trajectories/arc.hpp"
#include <iostream>

namespace autopilot {

Arc::Arc(const Eigen::Vector2d & start, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const double vehicle_speed, const bool clockwise_direction) : 
    StaticTrajectory(0.0, 1.0), vehicle_speed_(vehicle_speed), start_(start), center_(center), clockwise_direction_(clockwise_direction) {

    // ------------------------
    // Initialize the rotation matrix with the rotation 
    // information encoded in the normal vector
    // -----------------------
    normal_ = normal;
    
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

    // Set the clockwise direction variable to be -1 or 1
    clockwise_direction_ = (clockwise_direction) ? 1.0 : -1.0;

    // Compute the arc radius
    Eigen::Vector2d circle_planar_center(center[0], center[1]);
    radius_ = (circle_planar_center - start).norm();

    // Compute the angle of the starting point in the circle
    init_angle_ = std::atan2(start[1] - center[1], start[0] - center[0]);
}

Eigen::Vector3d Arc::pd(const double gamma) const {

    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * gamma * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d pd;
    pd[0] = radius_ * cos(curr_angle);
    pd[1] = radius_ * sin(curr_angle);
    pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    pd = rotation_ * pd;

    std::cout << "radius: " << radius_ << std::endl;
    std::cout << "curr_angle: " << curr_angle << std::endl;
    std::cout << "pd: " << pd.transpose() << std::endl;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return pd + center_;
}

Eigen::Vector3d Arc::d_pd(const double gamma) const {
    
    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * gamma * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d d_pd;
    d_pd[0] = -radius_ * sin(curr_angle) * (-clockwise_direction_ * M_PI);
    d_pd[1] = radius_ * cos(curr_angle) * (-clockwise_direction_ * M_PI);
    d_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    d_pd = rotation_ * d_pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return d_pd + center_;
}

Eigen::Vector3d Arc::d2_pd(const double gamma) const {
    
    // Compute the angle of the arc corresponding to the point in 2D space, according to the parametric value
    double curr_angle = init_angle_ - clockwise_direction_ * gamma * M_PI;

    // Compute the location of the 2D arc in a plane centered around [x y, 0.0]
    Eigen::Vector3d dd_pd;
    dd_pd[0] = -radius_ * cos(curr_angle) * std::pow(-clockwise_direction_ * M_PI, 2);
    dd_pd[1] = -radius_ * sin(curr_angle) * std::pow(-clockwise_direction_ * M_PI, 2);
    dd_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    dd_pd = rotation_ * dd_pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return dd_pd + center_;
}


double Arc::yaw(const double gamma) const {
    
    // Get the current position
    Eigen::Vector3d pd = this->pd(gamma);

    // Compute the vector pointing to the center of the arc
    Eigen::Vector3d center_to_pd = center_ - pd;

    // Compute the angle between the vector pointing to the center of the arc and the x-axis
    return std::atan2(center_to_pd[1], center_to_pd[0]);
}


double Arc::d_yaw(const double gamma) const {

    // Make the derivative of the angle approximately zero
    return 0.0;
}

double Arc::vehicle_speed(const double gamma) const {
    return vehicle_speed_;
}

double Arc::vd(const double gamma) const {
    
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

void ArcFactory::initialize() {
    
    // Load the service topic from the parameter server
    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.ArcFactory.service", "path/add_arc");

    // Advertise the service to add a line to the path
    add_arc_service_ = node_->create_service<pegasus_msgs::srv::AddArc>(node_->get_parameter("autopilot.StaticTrajectoryManager.ArcFactory.service").as_string(), std::bind(&ArcFactory::arc_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void ArcFactory::arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, pegasus_msgs::srv::AddArc::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding arc to path. Speed: " << request->speed.parameters[0] << ", start: [" << request->start[0] << "," << request->start[1] << "], center: [" << request->center[0] << "," << request->center[1] << "," << request->center[2] << "], normal: [" << request->normal[0] << "," << request->normal[1] << "," << request->normal[2] << "], clockwise: " << request->clockwise_direction << ".");

    // Create a new arc
    Arc::SharedPtr arc = std::make_shared<Arc>(Eigen::Vector2d(request->start.data()), Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->speed.parameters[0], request->clockwise_direction);

    // Add the arc to the path
    this->add_trajectory_to_manager(arc);

    // Update the response
    response->success = true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::ArcFactory, autopilot::StaticTrajectoryFactory)