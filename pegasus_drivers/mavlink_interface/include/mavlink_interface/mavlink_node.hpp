/*
 * Copyright 2022 Marcelo Jacinto.
 *
 * This file is part of the pegasus package and subject to the license terms
 * in the top-level LICENSE file of the pegasus repository.
 */
/**
 * @brief MAVLink Node
 * @file mavlink_node.hpp
 * @author Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 */
#pragma once

// MAVSDK Library for interfacing with mavlink
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mocap/mocap.h>

// ROS2 Dependencies
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pegasus_msgs/msg/rpy.hpp"
#include "pegasus_msgs/msg/state.hpp"

// Pegasus custome messages to control the vehicle
#include "pegasus_msgs/msg/body_velocity_control.hpp"
#include "pegasus_msgs/msg/attitude_thrust_control.hpp"
#include "pegasus_msgs/msg/attitude_rate_thrust_control.hpp"
#include "pegasus_msgs/msg/actuator_control.hpp"
#include "pegasus_msgs/msg/position_control.hpp"
#include "pegasus_msgs/msg/inertial_acceleration_control.hpp"

// Pegasus costume actions for arming/disarming, takeoff and landing the vehicle
#include "pegasus_msgs/visibility_control.h"
#include "pegasus_msgs/action/vehicle_land.hpp"
#include "pegasus_msgs/action/vehicle_take_off.hpp"
#include "pegasus_msgs/action/vehicle_arm.hpp"

// Pegasus costume services for emergency kill switch
#include "pegasus_msgs/srv/kill_switch.hpp"

// Pegasus costume messages to get vehicle sensor data
#include "pegasus_msgs/msg/status.hpp"

/**
 * @brief 
 * 
 */
class MAVLinkNode : public rclcpp::Node {
    public:

        /**
         * @brief Construct a new MAVLinkNode object
         */
        PEGASUS_MSGS_CPP_PUBLIC
        MAVLinkNode();

        /**
         * @brief Destroy the MAVLinkNode object
         */
        ~MAVLinkNode();

    private:

        /**
         * @brief Callback that is called periodically by ROS2 timer
         */
        void timer_callback();

        /**
         * @brief A MavSDK object
         */
        mavsdk::Mavsdk mavsdk_;

        /**
         * @brief The quadrotor system detected by mavsdk
         */
        std::shared_ptr<mavsdk::System> system_{nullptr};

        /**
         * @brief Object that will be used to request arming and disarm to mavlink
         */
        std::unique_ptr<mavsdk::Action> action_{nullptr};

        /**
         * @brief Object that will be used to send vehicle control command through mavlink
         */
        std::unique_ptr<mavsdk::Offboard> offboard_{nullptr};

        /**
         * @brief Object that will be used to subscribe to vehicle sensors from mavlink
         */
        std::unique_ptr<mavsdk::Telemetry> telemetry_{nullptr};

        /**
         * @brief Object that will be used to send vehicle position from MOCAP to autopilot through mavlink
         */
        std::unique_ptr<mavsdk::Mocap> mocap_{nullptr};

        /**
         * @brief Get the system object
         * 
         * @return std::shared_ptr<mavsdk::System> 
         */
        std::shared_ptr<mavsdk::System> get_system();

        /**
         * @brief A timer that periodically sends the data received by mavlink to ROS2
         */
        rclcpp::TimerBase::SharedPtr timer_;

        /**************************************
         * MAVLink Subscribers and Publishers
         **************************************/
        pegasus_msgs::msg::Status status_msg_;
        
        // Messages for the state of the vehicle, received by the EKF
        pegasus_msgs::msg::State state_msg_;

        // Messages for the IMU
        sensor_msgs::msg::Imu imu_msg_;
        sensor_msgs::msg::MagneticField magnetic_field_msg_;

        bool starting_offboard_{false};
        rclcpp::TimerBase::SharedPtr offboard_check_timer_;

        void initializeMAVLinkSubscribers();
        void armStateCallback(bool is_armed);

        /**
         * @brief Get the current state of the vehicle (current flight mode),
         * battery percentage, RC status and general health
         * 
         * @param flight_mode 
         */
        void flightModeCallback(mavsdk::Telemetry::FlightMode flight_mode);
        void batteryCallback(mavsdk::Telemetry::Battery battery);
        void rcStatusCallback(mavsdk::Telemetry::RcStatus rc_status);
        void healthCallback(mavsdk::Telemetry::Health health);

        /**
         * @brief Get the Attitude of the vehicle given by the EKF
         * as well as angular velocity (all expressed in NED)
         */
        void positionVelocityNEDCallback(mavsdk::Telemetry::PositionVelocityNed pos_vel_ned);
        void attitudeNEDQuaternion(mavsdk::Telemetry::Quaternion quat);
        void attitudeNEDEuler(mavsdk::Telemetry::EulerAngle euler);
        void angularVelocityBody(mavsdk::Telemetry::AngularVelocityBody ang_vel);
        void odometry(mavsdk::Telemetry::Odometry odom);

        /**
         * @brief Get the position
         * 
         * @param pos_vel_ned 
         */
        void imuCallback(mavsdk::Telemetry::Imu imu);

        /**************************************
         * ROS2 Setup
         **************************************/

        /**
         * @brief Method that is used to initialize all the ROS publishers (initialize state publishers, sensors publishers, among others)
         */
        void initializeROSPublishers();

        /**
         * @brief ROS Publisher for the status of the vehicle (connection, arm/disarm state, ...)
         */
        rclcpp::Publisher<pegasus_msgs::msg::Status>::SharedPtr status_pub_;
        rclcpp::Publisher<pegasus_msgs::msg::State>::SharedPtr state_pub_;

        // IMU Sensor data publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_pub_;

        /**
         * @brief Method that is used to initialize all the ROS subscribers (initialize subscribers for angular velocity control, angle control, among others)
         */
        void initializeROSSubscribers();

        rclcpp::Subscription<pegasus_msgs::msg::BodyVelocityControl>::SharedPtr body_velocity_sub_;
        rclcpp::Subscription<pegasus_msgs::msg::AttitudeThrustControl>::SharedPtr attitude_thrust_sub_;
        rclcpp::Subscription<pegasus_msgs::msg::AttitudeRateThrustControl>::SharedPtr attitude_rate_thrust_sub_;
        rclcpp::Subscription<pegasus_msgs::msg::ActuatorControl>::SharedPtr actuator_control_sub_;
        rclcpp::Subscription<pegasus_msgs::msg::PositionControl>::SharedPtr position_control_sub_;
        rclcpp::Subscription<pegasus_msgs::msg::InertialAccelerationControl>::SharedPtr acceleration_control_sub_;

        void bodyVelocityCallback(const pegasus_msgs::msg::BodyVelocityControl::SharedPtr msg);
        void attitudeThrustCallback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg);
        void attitudeRateThrustCallback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg);
        void actuatorControlCallback(const pegasus_msgs::msg::ActuatorControl::SharedPtr msg);
        void positionControlCallback(const pegasus_msgs::msg::PositionControl::SharedPtr msg);
        void inertialAccelerationCallback(const pegasus_msgs::msg::InertialAccelerationControl::SharedPtr msg);

        /**
         * @brief Method that is used to initialize the MOCAP ROS2 Subscriber
         */
        void initializeROSMocapSubscribers();
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_enu_sub_;
        void mocapPoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**************************************
         * ROS2 Services
         **************************************/
         
        /**
         * @brief Method that is used to initialize all the ROS services (TODO - define which services make sense - setting up gains?)
         */
        void initializeROSSevices();

        rclcpp::Service<pegasus_msgs::srv::KillSwitch>::SharedPtr kill_switch_service_;
        void kill_switch(const std::shared_ptr<pegasus_msgs::srv::KillSwitch::Request> request, const std::shared_ptr<pegasus_msgs::srv::KillSwitch::Response> response);

        /**************************************
         * ROS2 Actions
         **************************************/

        /**
         * @brief Method that is used to initialize all the ROS actions (initialize the arming, takoff action, landing, etc.)
         */
        void initializeROSActions();

        rclcpp_action::Server<pegasus_msgs::action::VehicleTakeOff>::SharedPtr take_off_action_server_;
        rclcpp_action::GoalResponse takeoff_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pegasus_msgs::action::VehicleTakeOff::Goal> goal);
        rclcpp_action::CancelResponse takeoff_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleTakeOff>> goal_handle);
        void takeoff_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleTakeOff>> goal_handle);

        rclcpp_action::Server<pegasus_msgs::action::VehicleLand>::SharedPtr land_action_server_;
        rclcpp_action::GoalResponse land_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pegasus_msgs::action::VehicleLand::Goal> goal);
        rclcpp_action::CancelResponse land_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleLand>> goal_handle);
        void land_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleLand>> goal_handle);

        rclcpp_action::Server<pegasus_msgs::action::VehicleArm>::SharedPtr arm_disarm_action_server_;
        rclcpp_action::GoalResponse arm_disarm_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pegasus_msgs::action::VehicleArm::Goal> goal);
        rclcpp_action::CancelResponse arm_disarm_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleArm>> goal_handle);
        void arm_disarm_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleArm>> goal_handle);


        /************************************************ 
         * AUXILIARY FUNCTIONS TO SWITCH MODES AUTOMATICALLY 
         **********************************************/
        void checkAndSwitchOffboardMode();
};