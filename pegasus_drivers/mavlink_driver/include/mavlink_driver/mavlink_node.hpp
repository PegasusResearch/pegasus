#pragma once

#include "ros_node.fwd.hpp"
#include "mavlink_node.fwd.hpp"

#include <chrono>
#include <atomic>
#include <Eigen/Core>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mocap/mocap.h>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief 
 */
class MavlinkNode {

public:
    
    /**
     * @brief Construct a new Mavlink Node object
     * @param nh The ROS2 nodehandler necessary to read parameters from the node parameter server
     * @param ros_node The ros_node object that will contain all the publishers and subscribers for ROS
     */
    MavlinkNode(const rclcpp::Node::SharedPtr nh, const std::shared_ptr<ROSNode> ros_node);

    /**
     * @brief Destroy the Mavlink Node object
     */
    ~MavlinkNode();

    /**
     * @defgroup control_callbacks
     * This group defines all the public control api callbacks for the vehicle
     */

    /**
     * @ingroup control_callbacks
     * @brief Method to set the body velocity of the vehicle expressed in f.r.d frame and yaw-rate
     * @param forward The forward body velocity in m/s
     * @param right The right body velocity in m/s
     * @param down The down body velocity in m/s
     * @param yaw_rate The yaw rate of the vehicle expressed in degres/s
     */
    void set_body_velocity(const float forward, const float right, const float down, const float yaw_rate);

    /**
     * @ingroup control_callbacks
     * @brief Method to set the attitude (roll, pitch and yaw) according to Z-Y-X convention and
     * expressed in degrees and thrust normalized thrust of the vehicle between 0-100%. The adopted
     * frame is f.r.d
     * @param roll The roll angle in deg
     * @param pitch The pitch angle in deg
     * @param yaw The yaw angle in deg
     * @param thrust The total thrust to apply to the vehicle, normalized between 0-100%
     */
    void set_attitude(const float roll, const float pitch, const float yaw, const float thrust);
    
    /**
     * @ingroup control_callbacks
     * @brief Method to set the attitude_rate (roll-rate, pitch-rate and yaw-rate) according to Z-Y-X convention
     * expressed in degrees-per-second and thrust normalized between 0-100%. The adopted
     * frame is f.r.d
     * @param roll_rate The roll rate in deg/s
     * @param pitch_rate The pitch rate in deg/s
     * @param yaw_rate The yaw rate in deg/s
     * @param thrust The total thrust to apply to the vehicle, normalized between 0-100%
     */
    void set_attitude_rate(const float roll_rate, const float pitch_rate, const float yaw_rate, const float thrust);

    /**
     * @ingroup control_callbacks
     * @brief Method to set the position (X-Y-Z) of the vehicle. The adopted frame is NED
     * @param x The North position in the inertial NED frame
     * @param y The East position in the inertial NED frame
     * @param z The Down position in the inertial NED frame
     * @param yaw The yaw angle in deg
     */
    void set_position(const float x, const float y, const float z, const float yaw);

    /**
     * @brief Method to arm or disarm the vehicle
     * @param arm_disarm The boolean that is 1 to arm the vehicle and 0 to disarm
     */
    void arm_disarm(const bool arm_disarm);

    /**
     * @brief Method to autoland the vehicle using the onboard microntroller controller
     */
    void land();

    /**
     * @brief Method to reboot the vehicle onboard microcontroller (NOT THE ONBOARD PC!)
     */
    void reboot();

    /**
     * @defgroup mocap
     * This group defines all the constants and callbacks used to receive data from a mocap system (if available) and send
     * to the onboard vehicle microcontrol for data fusion
     */

    /**
     * @ingroup mocap
     * @brief Method that is called whenever a new motion capture message is received from the network. This callback
     * will then send through mavlink to the onboard vehicle microcontrol for data fusion in the internal EKF of the vehicle
     * @param position The position of the body reference frame of the vehicle in (f.r.d) relative to the NED inertial reference frame
     * @param attitude A vector that expressed the orientation of the body reference frame of the vehicle (f.r.d) relative to the NED inertial frame
     * using roll, pitch and yaw angles expressed in radians according to a Z-Y-X rotation order
     */
    void update_mocap_telemetry(const Eigen::Vector3d &position, const Eigen::Vector3d &attitude);

private:

    /**
     * @defgroup system_initializations
     * This group defines all the public control api callbacks for the vehicle
     */

    /**
     * @ingroup system_initializations
     * @brief Method that is called whenever a new system is detected in the specified address by the mavsdk
     */
    void new_mavlink_system_callback();

    /**
     * @ingroup system_initializations
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize all
     * the telemetry mavlink subscriptions and subsequent ROS2 publishers for vehicle data/state/status
     */
    void initialize_telemetry();

    /**
     * @ingroup system_initializations
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize all
     * the actions that we can send through mavlink and create the corresponding ROS2 actions for arming/disarming and auto-landing
     */
    void initialize_actions();

    /**
     * @ingroup system_initializations
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detect to initialize all
     * the offboard mode controllers that can be used via mavlink and create the corresponding ROS2 subscribers
     * for receiving such control inputs
     */
    void initialize_offboard();

    /**
     * @ingroup system_initializations
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detect to initialize the
     * mocap mavsdk submodule and allow for publishing mocap data to be fused by the vehicle's onboard EFK filter
     */
    void initialize_mocap();

    /**
     * @ingroup system_initializations
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize
     * all mavlink redirections (like mavlink router) to the ips speficied in "mavlink_forward" ROS parameter
     */
    void initialize_mavlink_forwarding();

    /**
     * @defgroup offboard_mode
     * This group defines all the methods that switch on and off the OFFBOARD autopilot mode
     */

    /**
     * @ingroup offboard_mode
     * @brief Method that checks if the OFFBOARD autopilot is engaged, in order to send low-level
     * control command for the onboard micro-controller. If not, this method will send a signal
     * for the micro-controller to engage offboard mode
     */
    void check_switch_offboard_mode();

    /**
     * @defgroup vehicle_state_callbacks
     * This group defines all the methods that are called whenever a given state of the vehicle
     * is received via mavlink
     */

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current armed status of the vehicle
     * @param is_armed A boolean that is true if the vehicle is armed
     */
    void armed_state_callback(bool is_armed);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current engaged fligh mode of the vehicle
     * @param flight_mode A mavsdk structure that contains all possible flight modes
     */
    void flightmode_callback(mavsdk::Telemetry::FlightMode flight_mode);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current battery status of the vehicle
     * @param battery A mavsdk structure that contains the id of the battery, voltage and percentage
     */
    void battery_callback(mavsdk::Telemetry::Battery battery);
    
    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current state of the RC controller of the vehicle
     * @param rc_status A mavsdk structure that contains data whether an RC controller is paired with the vehicle
     * and the corresponding signal strength
     */
    void rc_status_callback(mavsdk::Telemetry::RcStatus rc_status);
    
    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current health of the vehicle and sensors
     * @param health A mavsdk structure that contains data regarding IMU calibration, local positon, global position
     * and home position definition state and ARMED/DISARMED state of the vehicle
     */
    void health_callback(mavsdk::Telemetry::Health health);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the current state of the imu
     * @param imu A mavsdk structure that containts data regarding the angular velocity, linear acceleration and
     * magnetic field (received in Gauss)
     */
    void imu_callback(mavsdk::Telemetry::Imu imu);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the position and velocity of the vehicle
     * expressed in the NED inertial frame of reference
     * @param pos_vel_ned A mavsdk structure that containts the position and linear velocity of the vehicle
     * expressed in the NED inertial frame of reference
     */
    void position_velocity_ned_callback(mavsdk::Telemetry::PositionVelocityNed pos_vel_ned);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the attitude of the vehicle body reference frame expressed in (f.r.d)
     * with respect to the inertial frame in NED in the inertial frame of reference, using quaternion
     * @param quat A mavsdk quaternion structure
     */
    void attitude_ned_quaternion_callback(mavsdk::Telemetry::Quaternion quat);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the attitude of the vehicle body reference frame expressed in (f.r.d)
     * with respect to the inertial frame in NED in the inertial frame of reference using Euler angles, according to a Z-Y-X
     * convention
     * @param euler A mavsdk structure with the euler angles roll, pitch and yaw expressed in degrees
     */
    void attitude_ned_euler_callback(mavsdk::Telemetry::EulerAngle euler);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the angular velocity of the vehicle body reference frame expressed in (f.r.d)
     * using a Z-Y-X convention
     * @param ang_vel A mavsdk structure with the angle rate roll-rate, pitch-rate and yaw-rate expressed in degrees/second
     */
    void angular_velocity_body_callback(mavsdk::Telemetry::AngularVelocityBody ang_vel);

    /**
     * @ingroup vehicle_state_callbacks
     * @brief Method that is called periodically to update the state of the vehicle expressed in the body reference frame (f.r.d)
     * @param odom A mavsdk odom structure for which we only care about the linear body velocity of the vehicle
     * expressed in the body frame of reference (f.r.d)
     */
    void odometry_callback(mavsdk::Telemetry::Odometry odom);

    /**
     * @brief A MavSDK object
     */
    mavsdk::Mavsdk mavsdk_;

    /**
     * @brief mavSDK configuration object, that sets hearbeat sending,
     * component and system ids of the onboard computer, etc.
     */
    std::shared_ptr<mavsdk::Mavsdk::Configuration> configuration_{nullptr};

    /**
     * @defgroup mavsdk_submodules
     * This group defines all the active mavsdk submodules
     */

    /**
     * @ingroup mavsdk_submodules
     * @brief The quadrotor system detected by mavsdk
     */
    std::shared_ptr<mavsdk::System> system_{nullptr};

    /**
     * @ingroup mavsdk_submodules
     * @brief Object that will be used to request arming and disarm to mavlink
     */
    std::unique_ptr<mavsdk::Action> action_{nullptr};

    /**
     * @ingroup mavsdk_submodules
     * @brief Object that will be used to send vehicle control command through mavlink
     */
    std::unique_ptr<mavsdk::Offboard> offboard_{nullptr};

    /**
     * @ingroup mavsdk_submodules
     * @brief Object that will be used to subscribe to vehicle sensors from mavlink
     */
    std::unique_ptr<mavsdk::Telemetry> telemetry_{nullptr};

    /**
     * @ingroup mavsdk_submodules
     * @brief Object that will be used to send vehicle position from MOCAP to autopilot through mavlink
     */
    std::unique_ptr<mavsdk::Mocap> mocap_{nullptr};

    /**
     * @defgroup mavsdk_control_messages
     * This group defines all the constant messages used by the mavlink node
     * that whose values are updated in real time
     */

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the body velocity of the vehicle expressed in f.r.d frame and yaw-rate
     */
    mavsdk::Offboard::VelocityBodyYawspeed body_velocity_and_yaw_rate_deg_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the attitude (roll, pitch and yaw) according to Z-Y-X convention and
     * expressed in degrees and thrust normalized thrust of the vehicle between 0-100%. The adopted
     * frame is f.r.d
     */
    mavsdk::Offboard::Attitude attitude_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the attitude_rate (roll-rate, pitch-rate and yaw-rate) according to Z-Y-X convention
     * expressed in degrees-per-second and thrust normalized between 0-100%. The adopted
     * frame is f.r.d
     */
    mavsdk::Offboard::AttitudeRate attitude_rate_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the position (X-Y-Z) of the vehicle. The adopted frame is NED
     */
    mavsdk::Offboard::PositionNedYaw position_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Bolean that is set to true when we send a request to start the offboard mode. After "check_offboard_status_timeout_" seconds,
     * this variable will be set to false again
     */
    bool starting_offboard_ = false;
    
    /**
     * @brief Duration variable used as a timeout to check whether offboard was engaged or not. During this time, we are not
     * able to engage offboard, even if the first trial failed
     */
    std::chrono::seconds check_offboard_status_timeout_{3};

    rclcpp::TimerBase::SharedPtr offboard_check_timer_;

    /**
     * @ingroup mocap
     * @brief Message to set the position (X-Y-Z) and attitude (roll, pitch and yaw) of the vehicle, where the body frame of the vehicle
     * is expressed according to the (f.r.d) reference frame and the inertial frame is expressed in NED frame. By default we initialize
     * the covariancle of the position and attitude states to NAN, so that the EKF used by the onboard controller uses the internal parameters
     * for computing the covariance. Moreover, the time is not specified such that the delay assumed between communications is also the one
     * defined by the internal parameters of the microcontroller.
     */
    mavsdk::Mocap::VisionPositionEstimate mocap_pose_;

    /**
     * @brief A ROS2 nodehandler that allows for reading parameters 
     * from the parameter server, along publishing to ROS2 topics
     */
    rclcpp::Node::SharedPtr nh_{nullptr};

    /**
     * @brief A ROS2node object that allows for initializing the ROS2 publishers, subscribers, etc.
     */
    std::shared_ptr<ROSNode> ros_node_{nullptr};
};