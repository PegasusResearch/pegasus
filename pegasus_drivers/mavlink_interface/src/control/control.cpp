/*
 * Copyright 2022 Marcelo Jacinto.
 *
 * This file is part of the pegasus package and subject to the license terms
 * in the top-level LICENSE file of the pegasus repository.
 */
/**
 * @brief MAVLink Node
 * @file control.cpp
 * @author Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 */

#include "rclcpp/rclcpp.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include "mavlink_node.hpp"

/**************************************
 * ROS2 Subscribers (for control)
 **************************************/

/**
 * @brief Method that is used to initialize all the ROS subscribers (initialize subscribers for angular velocity control, angle control, among others)
 */
void MAVLinkNode::initializeROSSubscribers() {

    // Subscribe to velocity control (in body frame NED) and desired yaw-angle
    this->declare_parameter<std::string>("subscribers.control.body_velocity", "control/body_velocity");
    rclcpp::Parameter body_velocity_topic = this->get_parameter("subscribers.control.body_velocity");
    this->body_velocity_sub_ = this->create_subscription<pegasus_msgs::msg::BodyVelocityControl>(
        body_velocity_topic.as_string(), 1, std::bind(&MAVLinkNode::bodyVelocityCallback, this, std::placeholders::_1));

    // Subscribe to the attitude (roll, pitch, yaw NED frame) and desired total thrust (0-100%)
    this->declare_parameter<std::string>("subscribers.control.attitude_thrust", "control/attitude_thrust");
    rclcpp::Parameter attitude_thrust_topic = this->get_parameter("subscribers.control.attitude_thrust");
    this->attitude_thrust_sub_ = this->create_subscription<pegasus_msgs::msg::AttitudeThrustControl>(
        attitude_thrust_topic.as_string(), 1, std::bind(&MAVLinkNode::attitudeThrustCallback, this, std::placeholders::_1));

    // Subscribe to the attitude rate (roll-rate, pitch-rate, yaw-rate in NED frame) and desired total thrust (0-100%)
    this->declare_parameter<std::string>("subscribers.control.attitude_rate_thrust", "control/attitude_rate_thrust");
    rclcpp::Parameter attitude_rate_thrust_topic = this->get_parameter("subscribers.control.attitude_rate_thrust");
    this->attitude_rate_thrust_sub_ = this->create_subscription<pegasus_msgs::msg::AttitudeRateThrustControl>(
        attitude_rate_thrust_topic.as_string(), 1, std::bind(&MAVLinkNode::attitudeRateThrustCallback, this, std::placeholders::_1));

    // Subscribe to the individual actuator control values
    this->declare_parameter<std::string>("subscribers.control.actuator_control", "control/actuator_control");
    rclcpp::Parameter actuator_control_topic = this->get_parameter("subscribers.control.actuator_control");
    this->actuator_control_sub_ = this->create_subscription<pegasus_msgs::msg::ActuatorControl>(
        actuator_control_topic.as_string(), 1, std::bind(&MAVLinkNode::actuatorControlCallback, this, std::placeholders::_1));

    // Subscribe to the position control (north-east-down in meters NED) and desired yaw (in deg)
    this->declare_parameter<std::string>("subscribers.control.position", "control/position");
    rclcpp::Parameter position_control_topic = this->get_parameter("subscribers.control.position");
    this->position_control_sub_ = this->create_subscription<pegasus_msgs::msg::PositionControl>(
        position_control_topic.as_string(), 1, std::bind(&MAVLinkNode::positionControlCallback, this, std::placeholders::_1));

    // Subscribe to the desired acceleration expressed in the inertial frame (NED)
    this->declare_parameter<std::string>("subscribers.control.inertial_acceleration", "control/inertial_acceleration");
    rclcpp::Parameter inertial_acceleration_control_topic = this->get_parameter("subscribers.control.inertial_acceleration");
    this->acceleration_control_sub_ = this->create_subscription<pegasus_msgs::msg::InertialAccelerationControl>(
        inertial_acceleration_control_topic.as_string(), 1, std::bind(&MAVLinkNode::inertialAccelerationCallback, this, std::placeholders::_1));
}

/**
 * @brief TODO
 * 
 * @param msg 
 */
void MAVLinkNode::bodyVelocityCallback(const pegasus_msgs::msg::BodyVelocityControl::SharedPtr msg) {
    
    // Populate the mavsdk structure (with body velocity in m/s and yaw-rate in deg) and send it through mavlink
    mavsdk::Offboard::VelocityBodyYawspeed body_velocity_and_yaw_rate_deg{};

    body_velocity_and_yaw_rate_deg.forward_m_s = msg->velocity_body[0];
    body_velocity_and_yaw_rate_deg.right_m_s = msg->velocity_body[1];
    body_velocity_and_yaw_rate_deg.down_m_s = msg->velocity_body[2];
    body_velocity_and_yaw_rate_deg.yawspeed_deg_s = msg->yaw_rate_deg;

    this->offboard_->set_velocity_body(body_velocity_and_yaw_rate_deg);

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief TODO
 * 
 * @param msg 
 */
void MAVLinkNode::attitudeThrustCallback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg) {

    // Populate the mavsdk structure (with attitude - roll, pitch and yaw in deg, following Z-Y-X convention) and 
    // total thrust scaled between 0-100%, converted to 0<->1 and send it through mavlink
    mavsdk::Offboard::Attitude attitude{};

    attitude.roll_deg = msg->attitude[0];
    attitude.pitch_deg = msg->attitude[1];
    attitude.yaw_deg = msg->attitude[2];
    // Make sure the thrust value that get's requested is normalized between 0 and 1
    attitude.thrust_value = std::min(std::max(msg->thrust / 100.0, 0.0), 1.0); 
    this->offboard_->set_attitude(attitude);

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief TODO
 * 
 * @param msg 
 */
void MAVLinkNode::attitudeRateThrustCallback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg) {

    // Populate the mavsdk structure (with attitude rate - roll-rate, pitch-rate, yaw-rate in deg/sec, following the Z-Y-X convention) and
    // total thrust scaled between 0-100%, converted to 0<->1 and send it through mavlink
    mavsdk::Offboard::AttitudeRate attitude_rate{};

    attitude_rate.roll_deg_s = msg->attitude_rate[0];
    attitude_rate.pitch_deg_s = msg->attitude_rate[1]; 
    attitude_rate.yaw_deg_s = msg->attitude_rate[2];
    // Make sure the thrust value that get's requested is normalized between 0 and 1
    attitude_rate.thrust_value = std::min(std::max(msg->thrust / 100.0, 0.0), 1.0); 
    this->offboard_->set_attitude_rate(attitude_rate);

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief TODO - check this function with care - TO BE TESTED PROPERLY!
 * 
 * @param msg 
 */
void MAVLinkNode::actuatorControlCallback(const pegasus_msgs::msg::ActuatorControl::SharedPtr msg) {
    
    // The typical group for the thrusters (max number of actuators per group - 8 [0...7])
    mavsdk::Offboard::ActuatorControlGroup group0{};

    // The typical group for gimbal and lights (max number of actuators per group -8 [0...7])
    mavsdk::Offboard::ActuatorControlGroup group1{};

    // Set the control inputs for the thrusters based on the ROS2 message received 
    std::move(msg->group0.begin(), msg->group0.begin() + 8, group0.controls.begin());

    // Set the control inputs for the gimbal, lights, etc. based on the ROS2 message received 
    // TODO: this group is not tested - Use with care!!!
    std::move(msg->group1.begin(), msg->group1.begin() + 8, group1.controls.begin());

    this->offboard_->set_actuator_control(mavsdk::Offboard::ActuatorControl{{group0, group1}});

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief TODO
 * 
 * @param msg 
 */
void MAVLinkNode::positionControlCallback(const pegasus_msgs::msg::PositionControl::SharedPtr msg) {

    // Populate the MAVSDK structure (with position X, Y, Z in inertial NED frame -> north-east-down) and the respective yaw angle (expressed in deg)
    mavsdk::Offboard::PositionNedYaw position{};
    position.north_m = msg->position[0];
    position.east_m = msg->position[1];
    position.down_m = msg->position[2];
    position.yaw_deg = msg->yaw;

    this->offboard_->set_position_ned(position);

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief TODO
 * 
 * @param msg 
 */
void MAVLinkNode::inertialAccelerationCallback(const pegasus_msgs::msg::InertialAccelerationControl::SharedPtr msg) {

    // Populate the MAVSDK structure (with acceleration in NED expressed in m/s^2)
    mavsdk::Offboard::AccelerationNed acceleration_ned;
    acceleration_ned.north_m_s2 = msg->inertial_acceleration[0];
    acceleration_ned.east_m_s2 = msg->inertial_acceleration[1];
    acceleration_ned.down_m_s2 = msg->inertial_acceleration[2];

    this->offboard_->set_acceleration_ned(acceleration_ned);

    // Check if the vehicle is already in offboard mode - if not, then 
    this->checkAndSwitchOffboardMode();
}

/**
 * @brief 
 */
void MAVLinkNode::checkAndSwitchOffboardMode() {

    // Check if are on offboard mode already, if not, then switch to it
    if(this->status_msg_.flight_mode != this->status_msg_.OFFBOARD && !this->starting_offboard_) {
        this->starting_offboard_ = true;

        using namespace std::chrono_literals;
        // Initiate a timer to check 3 seconds later and make the starting offboard variable false
        offboard_check_timer_ = this->create_wall_timer(3.0s, [this] () {
            this->starting_offboard_ = false;
            this->offboard_check_timer_->cancel();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Starting offboard: " << this->starting_offboard_);
        });

        // Start the offboard mode and signal the user
        this->offboard_->start_async([](mavsdk::Offboard::Result result) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Offboard mode: " << result);
        });
    }
}