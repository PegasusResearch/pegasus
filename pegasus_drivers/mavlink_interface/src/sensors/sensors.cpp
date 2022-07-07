/*
 * Copyright 2022 Marcelo Jacinto.
 *
 * This file is part of the pegasus package and subject to the license terms
 * in the top-level LICENSE file of the pegasus repository.
 */
/**
 * @brief MAVLink Node
 * @file sensors.cpp
 * @author Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 */

#include "rclcpp/rclcpp.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "mavlink_node.hpp"

/**************************************
 * MAVLink Subscribers and Publishers
 **************************************/

/**
 * @brief TODO
 */
void MAVLinkNode::initializeMAVLinkSubscribers() {

    // Subscribe to the current arm state, flight mode, battery level, RC strength of the vehicle
    this->telemetry_->subscribe_armed(std::bind(&MAVLinkNode::armStateCallback, this, std::placeholders::_1));
    this->telemetry_->subscribe_flight_mode(std::bind(&MAVLinkNode::flightModeCallback, this, std::placeholders::_1));
    this->telemetry_->subscribe_battery(std::bind(&MAVLinkNode::batteryCallback, this, std::placeholders::_1));
    this->telemetry_->subscribe_rc_status(std::bind(&MAVLinkNode::rcStatusCallback, this, std::placeholders::_1));
    this->telemetry_->subscribe_health(std::bind(&MAVLinkNode::healthCallback, this, std::placeholders::_1));

    // Subscribe to the raw data from the imu
    this->telemetry_->set_rate_imu(20);
    this->telemetry_->subscribe_imu(std::bind(&MAVLinkNode::imuCallback, this, std::placeholders::_1));

    // Subscribe to the position/velocity in the NED inertial frame
    this->telemetry_->set_rate_position_velocity_ned(20);
    this->telemetry_->subscribe_position_velocity_ned(std::bind(&MAVLinkNode::positionVelocityNEDCallback, this, std::placeholders::_1));

    // Subscribe to the filtered attitude of the vehicle
    this->telemetry_->set_rate_attitude(20);
    this->telemetry_->subscribe_attitude_quaternion(std::bind(&MAVLinkNode::attitudeNEDQuaternion, this, std::placeholders::_1));
    this->telemetry_->subscribe_attitude_euler(std::bind(&MAVLinkNode::attitudeNEDEuler, this, std::placeholders::_1));
    this->telemetry_->subscribe_attitude_angular_velocity_body(std::bind(&MAVLinkNode::angularVelocityBody, this, std::placeholders::_1));
    
    // Subscribe to the odometry message of the vehicle
    this->telemetry_->set_rate_odometry(20);
    this->telemetry_->subscribe_odometry(std::bind(&MAVLinkNode::odometry, this, std::placeholders::_1));
    
    // Subscribe to the actuator state of the vehicle
    //this->telemetry_->subscribe_actuator_control_target([](mavsdk::Telemetry::ActuatorControlTarget act_control_target) {std::cout << act_control_target << std::endl; });
    //this->telemetry_->subscribe_actuator_output_status([](mavsdk::Telemetry::ActuatorOutputStatus act_output) {std::cout << act_output << std::endl; });
}

/**
 * @brief Callback that is called when the arm state is received via mavlink.
 * It updates the Status message and publishes in ROS2
 * 
 * @param is_armed The arm state of the vehicle
 */
void MAVLinkNode::armStateCallback(bool is_armed) {
    this->status_msg_.header.stamp = rclcpp::Clock().now();
    this->status_msg_.armed = is_armed;
    this->status_pub_->publish(this->status_msg_);
}

/**
 * @brief Callback that is called when the current flight mode is received via mavlink
 * It updates the Status message and publishes in ROS2
 * 
 * @param flight_mode The fligh mode of the vehicle's micro-controller
 */
void MAVLinkNode::flightModeCallback(mavsdk::Telemetry::FlightMode flight_mode) {
    this->status_msg_.header.stamp = rclcpp::Clock().now();
    this->status_msg_.flight_mode = static_cast<unsigned char>(flight_mode);
    this->status_pub_->publish(this->status_msg_);
}

/**
 * @brief Callback that is called when the current battery percentage is received via mavlink
 * It upates the Status message and publishes in ROS2
 * 
 * @param battery The battery level of the vehicle
 */
void MAVLinkNode::batteryCallback(mavsdk::Telemetry::Battery battery) {
    this->status_msg_.header.stamp = rclcpp::Clock().now();
    this->status_msg_.battery = battery.remaining_percent * 100.0;
    this->status_pub_->publish(this->status_msg_);
}

/**
 * @brief Callback that is called when the current RC Joystick status is received via mavlink
 * It updates the Status message and publishes in ROS2. Values vary between [0,100] when the
 * RC is connected to the Pixhawk, and -1 if no RC is detected.
 * 
 * @param rc_status The RC Joystick object
 */
void MAVLinkNode::rcStatusCallback(mavsdk::Telemetry::RcStatus rc_status) {
    this->status_msg_.header.stamp = rclcpp::Clock().now();
    this->status_msg_.rc_signal = (rc_status.is_available) ? rc_status.signal_strength_percent * 100.0 : -1;
    this->status_pub_->publish(this->status_msg_);
}

/**
 * @brief Callback that is called when the current health of the vehicle is received via mavlink
 * It update the Status message (the field health) and publishes in ROS2.
 * 
 * @param health The Health object from mavsdk
 */
void MAVLinkNode::healthCallback(mavsdk::Telemetry::Health health) {
    this->status_msg_.header.stamp = rclcpp::Clock().now();
    this->status_msg_.health.is_armable = health.is_armable;
    this->status_msg_.health.accelerometer_calibrated = health.is_accelerometer_calibration_ok;
    this->status_msg_.health.magnetometer_calibrated = health.is_magnetometer_calibration_ok;

    // TODO - check if these 3 fields in the message is really necessary or if we can omit them for the sake of simplicity
    this->status_msg_.health.local_position_ok = health.is_local_position_ok;
    this->status_msg_.health.global_position_ok = health.is_global_position_ok;
    this->status_msg_.health.home_position_ok = health.is_home_position_ok;
    this->status_pub_->publish(this->status_msg_);
}

/**
 * @brief Callback that is called when the current PositionVelocityNED is received via mavlink
 * It updates the PoseStamped message (the field position) and publishes in ROS2.
 *
 * @param pos_vel_ned The PositionVelocityNed object from MAVSDK
 */
void MAVLinkNode::positionVelocityNEDCallback(mavsdk::Telemetry::PositionVelocityNed pos_vel_ned) {

    // Update the absolule position of the vehicle in the "pose" message
    this->state_msg_.pose.header.stamp = rclcpp::Clock().now();
    this->state_msg_.pose.pose.position.x = pos_vel_ned.position.north_m;
    this->state_msg_.pose.pose.position.y = pos_vel_ned.position.east_m;
    this->state_msg_.pose.pose.position.z = pos_vel_ned.position.down_m;

    // Update the inertial velocity of the vehicle x_dot, y_dot, z_dot
    this->state_msg_.inertial_vel.header.stamp = rclcpp::Clock().now();
    this->state_msg_.inertial_vel.vector.x = pos_vel_ned.velocity.north_m_s;
    this->state_msg_.inertial_vel.vector.y = pos_vel_ned.velocity.east_m_s;
    this->state_msg_.inertial_vel.vector.z = pos_vel_ned.velocity.down_m_s;

    // Publish the messages to ROS2
    this->state_pub_->publish(this->state_msg_);
}

void MAVLinkNode::odometry(mavsdk::Telemetry::Odometry odom) {

    // Update the body velocity message
    this->state_msg_.body_vel.header.stamp = rclcpp::Clock().now();
    this->state_msg_.body_vel.twist.linear.x = odom.velocity_body.x_m_s;
    this->state_msg_.body_vel.twist.linear.y = odom.velocity_body.y_m_s;
    this->state_msg_.body_vel.twist.linear.z = odom.velocity_body.z_m_s;

    this->state_pub_->publish(this->state_msg_);
}

void MAVLinkNode::attitudeNEDQuaternion(mavsdk::Telemetry::Quaternion quat) {

    // Update the orientation of the vehicle in the pose message
    // Update the absolule position of the vehicle in the "pose" message
    this->state_msg_.pose.header.stamp = rclcpp::Clock().now();
    this->state_msg_.pose.pose.orientation.w = quat.w;
    this->state_msg_.pose.pose.orientation.x = quat.x;
    this->state_msg_.pose.pose.orientation.y = quat.y;
    this->state_msg_.pose.pose.orientation.z = quat.z;
    this->state_pub_->publish(this->state_msg_);
}

/**
 * @brief Callback that is called when data related to angular velocity is received via mavlink
 * It updates the TwistStamped message (the field angular velocity) and publishes ROS2.
 * 
 * @param ang_vel The angular velocity in rad/s
 */
void MAVLinkNode::angularVelocityBody(mavsdk::Telemetry::AngularVelocityBody ang_vel) {

    this->state_msg_.body_vel.header.stamp = rclcpp::Clock().now();
    this->state_msg_.body_vel.twist.angular.x = ang_vel.roll_rad_s;
    this->state_msg_.body_vel.twist.angular.y = ang_vel.pitch_rad_s;
    this->state_msg_.body_vel.twist.angular.z = ang_vel.yaw_rad_s;

    this->state_pub_->publish(this->state_msg_);
}

/**
 * @brief Callback attitude NED Euler
 * 
 * @param euler Roll, Pitch and Yaw in degrees
 */
void MAVLinkNode::attitudeNEDEuler(mavsdk::Telemetry::EulerAngle euler) {
    
    this->state_msg_.rpy.header.stamp = rclcpp::Clock().now();
    this->state_msg_.rpy.vector.x = euler.roll_deg;
    this->state_msg_.rpy.vector.y = euler.pitch_deg;
    this->state_msg_.rpy.vector.z = euler.yaw_deg;

    this->state_pub_->publish(this->state_msg_);
}

/**
 * @brief Callback that is called when an IMU message is received via mavlink
 * It updates the IMU and MagneticField messages and publishes them in ROS2
 * 
 * @param imu The Imu object from MAVSDK
 */
void MAVLinkNode::imuCallback(mavsdk::Telemetry::Imu imu) {
    this->imu_msg_.header.stamp = rclcpp::Clock().now();

    // TODO - put the orientation in quaternion somewhere here

    // Angular velocity measured directly by the IMU
    this->imu_msg_.angular_velocity.x = imu.angular_velocity_frd.forward_rad_s;
    this->imu_msg_.angular_velocity.y = imu.angular_velocity_frd.right_rad_s;
    this->imu_msg_.angular_velocity.z = imu.angular_velocity_frd.down_rad_s;

    // Linear acceleration measured directly by the IMU
    this->imu_msg_.linear_acceleration.x = imu.acceleration_frd.forward_m_s2;
    this->imu_msg_.linear_acceleration.y = imu.acceleration_frd.right_m_s2;
    this->imu_msg_.linear_acceleration.z = imu.acceleration_frd.down_m_s2;
    this->imu_pub_->publish(this->imu_msg_);

    // TODO: Check if this magnetic field (received in Gauss) is being correctly scaled to Tesla Units
    this->magnetic_field_msg_.header.stamp = this->imu_msg_.header.stamp;
    this->magnetic_field_msg_.magnetic_field.x = imu.magnetic_field_frd.forward_gauss;
    this->magnetic_field_msg_.magnetic_field.y = imu.magnetic_field_frd.right_gauss;
    this->magnetic_field_msg_.magnetic_field.z = imu.magnetic_field_frd.down_gauss;
    this->magnetic_field_pub_->publish(this->magnetic_field_msg_);
}
