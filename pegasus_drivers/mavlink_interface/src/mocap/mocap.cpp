/*
 * Copyright 2022 Marcelo Jacinto.
 *
 * This file is part of the pegasus package and subject to the license terms
 * in the top-level LICENSE file of the pegasus repository.
 */
/**
 * @brief MAVLink Node
 * @file mocap.cpp
 * @author Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 */

#include "rclcpp/rclcpp.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include "mavlink_node.hpp"
#include "frames.hpp"
#include <Eigen/Dense>

/**
 * @brief Method that is used to initialize the ROS subscriber for the MOCAP data
 */
void MAVLinkNode::initializeROSMocapSubscribers() {

    // Subscribe to velocity control (in body frame NED) and desired yaw-angle
    this->declare_parameter<std::string>("subscribers.external_sensors.mocap_enu", "mocap/pose_enu");
    rclcpp::Parameter mocap_enu_topic = this->get_parameter("subscribers.external_sensors.mocap_enu");
    this->mocap_pose_enu_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_enu_topic.as_string(), 1, std::bind(&MAVLinkNode::mocapPoseSubscriber, this, std::placeholders::_1));
}

/**
 * @brief Method that is used to subscribe to data config from the MOCAP system in ENU and
 * send a converted version to NED to the Micro-controller through mavlink
 * 
 * @param msg  A PoseStamped geometry message with the position and orientation of the vehicle
 */
void MAVLinkNode::mocapPoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    
    // Convert the position expressed in ENU {East-North-Up} to NED {North-East-Down}
    Eigen::Vector3d position_ned = Pegasus::Frames::transform_vect_inertial_enu_ned(
        Eigen::Vector3d(
            msg->pose.position.x, 
            msg->pose.position.y,
            msg->pose.position.z));
        
    // Convert the orientation of a F.L.U vehicle with respect to ENU {East-North-Up} to
    // F.R.D with respect to NED {North-East-Down}
    Eigen::Quaterniond orientation_flu_enu;
    orientation_flu_enu.x() = msg->pose.orientation.x;
    orientation_flu_enu.y() = msg->pose.orientation.y;
    orientation_flu_enu.z() = msg->pose.orientation.z;
    orientation_flu_enu.w() = msg->pose.orientation.w;

    Eigen::Vector3d orientation_frd_ned = Pegasus::Rotations::quaternion_to_euler(
        Pegasus::Frames::rot_body_to_inertial(
            Eigen::Quaternion<double>(orientation_flu_enu)));

    // Create the mavlink message to send to the vehicle
    static mavsdk::Mocap::VisionPositionEstimate mocap_pose;

    // Create the covariance matrix (where we specify that we do not know the covariance for position and orientation, using NANs)
    mavsdk::Mocap::Covariance mocap_covariance;
    mocap_covariance.covariance_matrix = std::vector<float>(21, 0.0);
    mocap_covariance.covariance_matrix[0] = NAN;
    mocap_covariance.covariance_matrix[15] = NAN;

    // Assign the covariance matrix to mocap pose message
    mocap_pose.pose_covariance = mocap_covariance;

    // Set the position of the vehicle in the inertial frame (expressed in NED)
    mocap_pose.position_body.x_m = position_ned.x();
    mocap_pose.position_body.y_m = position_ned.y();
    mocap_pose.position_body.z_m = position_ned.z();

    // Set the orientation of the vehicle in the inertial frame (expressed in NED)
    // with body frame expressed in f.r.d
    mocap_pose.angle_body.roll_rad = orientation_frd_ned.x();
    mocap_pose.angle_body.pitch_rad = orientation_frd_ned.y();
    mocap_pose.angle_body.yaw_rad = orientation_frd_ned.z();

    // Define the mocap time
    mocap_pose.time_usec = 0;

    // Send the current position to the vehicle through mavlink
    mavsdk::Mocap::Result result = this->mocap_->set_vision_position_estimate(mocap_pose);
    if (result != mavsdk::Mocap::Result::Success) RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), result);
}