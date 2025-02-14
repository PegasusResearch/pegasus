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
#pragma once

#include <chrono>
#include <atomic>
#include <Eigen/Core>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
//#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mocap/mocap.h>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief Mavlink interface wrapper using Mavsdk to communicate with the vehicle
 */
class MavlinkNode {

public:

    /**
     * @brief An alias for a shared, unique and weak pointers of a MavlinkNode object
     */
    using SharedPtr = std::shared_ptr<MavlinkNode>;
    using UniquePtr = std::unique_ptr<MavlinkNode>;
    using WeakPtr = std::weak_ptr<MavlinkNode>;

    struct MavlinkNodeConfig {

        std::string connection_address;                                    // The address of the vehicle to connect to (e.g. udp://:14540@localhost:14557)
        std::vector<std::string> forward_ips;                              // The ips to forward the mavlink messages to (e.g. QGroundControl)

        // Rates for the 
        double rate_attitude;
        double rate_position;
        double rate_gps;
        double rate_altitude;
        double rate_imu;
        double rate_distance;

        std::function<void(uint8_t)> on_discover_callback{nullptr};        // Callback to be called whenever a new system is discovered, which receives the vehicle id
        std::function<void()> on_initialize_telemetry_callback{nullptr};   // Callback to be called whenever telemetry coming from the vehicle is initialized
        std::function<void()> on_initialize_actions_callback{nullptr};     // Callback to be called whenever actions that can be sent to the vehicle are initialized

        // Callbacks for handling the Telemetry raw sensor data received by the vehicle (IMU, barometer and gps)
        std::function<void(const mavsdk::Telemetry::Imu &)> on_imu_callback{nullptr};                               // Callback to handle HIGHRES_IMU mavlink messages (RAW IMU data)
        std::function<void(const mavsdk::Telemetry::Altitude &)> on_altitude_callback{nullptr};                     // Callback to handle ALTITUDE mavlink messages (RAW altimeter data)
        std::function<void(const mavsdk::Telemetry::RawGps &)> on_raw_gps_callback{nullptr};                        // Callback to handle GPS_RAW_INT mavlink messages (GPS data)
        std::function<void(const mavsdk::Telemetry::GpsInfo &)> on_gps_info_callback{nullptr};                      // Callback to handle GPS_RAW_INT mavlink messages (GPS status)
        std::function<void(const mavsdk::Telemetry::DistanceSensor &)> on_distance_sensor_callback{nullptr};        // Callback to handle DISTANCE_SENSOR mavlink messages (Distance sensor data)

        // Callbacks for the filtered state of the vehicle (attitude, position and velocity) provided by EKF2
        std::function<void(const mavsdk::Telemetry::Quaternion &)> on_quaternion_callback{nullptr};                 // Callback to handle ATTITUDE_QUATERNION mavlink messages (Attitude + ang vel estimated by the complementary filter)
        std::function<void(const mavsdk::Telemetry::AngularVelocityBody &)> on_angular_velocity_callback{nullptr};  // Callback to handle ATTITUDE_QUATERNION mavlink messages (Attitude + ang vel estimated by the complementary filter)
        std::function<void(const mavsdk::Telemetry::PositionVelocityNed &)> on_position_velocity_callback{nullptr}; // Callback to handle LOCAL_POSITION_NED mavlink messages (Position + velocity estimated by the onboard EKF2)

        // Callbacks for handling the Status and operating modes of the vehicle
        std::function<void(const bool &)> on_armed_callback{nullptr};                                               // Callback to handle HEARTBEAT mavlink messages - armed state of the vehicle
        std::function<void(const mavsdk::Telemetry::LandedState &)> on_landed_state_callback{nullptr};              // Callback to handle EXTENDED_SYS_STATE mavlink messages - landed state of the vehicle
        std::function<void(const mavsdk::Telemetry::FlightMode &)> on_flight_mode_callback{nullptr};                // Callback to handle HEARTBEAT mavlink messages - flight mode of the vehicle
        std::function<void(const mavsdk::Telemetry::Health &)> on_health_callback{nullptr};                         // Callback to handle HEARTBEAT mavlink messages - health of the vehicle
        std::function<void(const mavsdk::Telemetry::Battery &)> on_battery_callback{nullptr};                       // Callback to handle BATTERY_STATUS mavlink messages - battery status of the vehicle
        std::function<void(const mavsdk::Telemetry::RcStatus &)> on_rc_callback{nullptr};                           // Callback to handle RC_CHANNELS mavlink messages - RC channels of the vehicle
    };
    
    /**
     * @brief Construct a new Mavlink Node object
     * @param config The configuration of the mavlink node
     */
    MavlinkNode(const MavlinkNodeConfig &config);

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
     * @ingroup control_callbacks
     * @brief Set the inertial velocity (Vx, Vy, Vz) (m/s) and yaw (deg) of the vehicle. The adopted frame is NED
     * @param vx The velocity in the North direction in the inertial NED frame
     * @param vy The velocity in the East direction in the inertial NED frame
     * @param vz The velocity in the Down direction in the inertial NED frame
     * @param yaw The yaw angle in deg
     */
    void set_inertial_velocity(const float vx, const float vy, const float vz, const float yaw);

    /**
     * @ingroup control_callbacks
     * @brief Set the body velocity (Vx, Vy, Vz) (m/s) and yaw-rate (deg/s) of the vehicle. The adopted frame is f.r.d
     * Note that with quadrotors, the roll and pitch of the body frame-are ignored, and the vehicle is actuated in a "cheated" body velocity
     * @param vx The velocity in the x direction in the body frame (projected for 0 roll)
     * @param vy The velocity in the y direction in the body frame (projected for 0 pitch)
     * @param vz The velocity in the z direction in the body frame
     * @param yaw_rate The angular velocity around the z-axis in the body frame
     */
    void set_body_velocity(const float vx, const float vy, const float vz, const float yaw_rate);

    /**
     * @ingroup control_callbacks
     * @brief Set the inertial acceleration (Ax, Ay, Az) (m/s^2) of the vehicle. The adopted frame is NED
     * @param ax The acceleration in the North direction in the inertial NED frame
     * @param ay The acceleration in the East direction in the inertial NED frame
     * @param az The acceleration in the Down direction in the inertial NED frame
     */
    void set_inertial_acceleration(const float ax, const float ay, const float az);

    /**
     * @brief Method to arm or disarm the vehicle
     * @param arm_disarm The boolean that is 1 to arm the vehicle and 0 to disarm
     */
    uint8_t arm_disarm(const bool arm_disarm);

    /**
     * @brief Method to kill the vehicle motors instantly
     */
    uint8_t kill_switch();

    /**
     * @brief Method to autoland the vehicle using the onboard microntroller controller
     */
    uint8_t land();

    /**
    * @brief Sends a signal to control a specific motor or actuator.
    * 
    * @param index Specifies the index of the motor/actuator, "Actuactor Set", to control.
    *              This should be an integer representing the target gate
    * @param value Specifies the value to set for the actuator
    *              The value should be a float, between -1.0 (minimum) and 1.0 (maximum)
    */
    uint8_t set_motors(const int index, const float value);
    
    /**
     * @brief Method to make the vehicle switch to the offboard mode. This function is blocking
     * until we have a result from the vehicle.
     */
    uint8_t offboard();

    /**
     * @brief Method to make the vehicle switch to the position hold mode. This function is blocking
     * until we have a result from the vehicle. 
     */
    uint8_t position_hold();

    /**
     * @brief Method to set the home position of the onboard micro-controller.
     */
    //void set_home_position();

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
     * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize the
     * mavlink passthrough submodule and allow for sending and receiving mavlink messages to and from the vehicle.
     */
    //void initialize_mavlink_passthrough();

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
     * @defgroup vehicle_state_callbacks
     * This group defines all the methods that are called whenever a given state of the vehicle
     * is received via mavlink
     */

    /**
     * @brief A MavSDK object and mavSDK configuration object, that sets hearbeat sending,
     * component and system ids of the onboard computer, etc.
     */
    mavsdk::Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::CompanionComputer}};
    mavsdk::Mavsdk::NewSystemHandle new_system_handle_;
    std::shared_ptr<mavsdk::Mavsdk::Configuration> configuration_{nullptr};

    /**
     * @brief The ID of the vehicle connected via mavlink and callback configurations for the mavlink_node
     */
    uint8_t system_id_;
    MavlinkNodeConfig config_;

    /**
     * @defgroup mavsdk_submodules
     * This group defines all the active mavsdk submodules
     */

    /**
     * @ingroup mavsdk_submodules
     * @brief The quadrotor system detected by mavsdk
     */
    std::shared_ptr<mavsdk::System> system_{nullptr};
    std::unique_ptr<mavsdk::Action> action_{nullptr};
    std::unique_ptr<mavsdk::Offboard> offboard_{nullptr};
    std::unique_ptr<mavsdk::Telemetry> telemetry_{nullptr};
    //std::unique_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough_{nullptr};
    std::unique_ptr<mavsdk::Mocap> mocap_{nullptr};    

    /**
     * @defgroup mavsdk_control_messages
     * This group defines all the constant messages used by the mavlink node
     * that whose values are updated in real time
     */

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
     * @brief Message to set the velocity (Vx, Vy, Vz) (m/s) and yaw (deg) of the vehicle. The adopted frame is NED
     */
    mavsdk::Offboard::VelocityNedYaw inertial_velocity_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the velocity in the body-frame (Vx, Vy, Vz) (m/s) and yaw-rate (deg/s) of the vehicle. The adopted frame is f.r.d
     */
    mavsdk::Offboard::VelocityBodyYawspeed body_velocity_;

    /**
     * @ingroup mavsdk_control_messages
     * @brief Message to set the acceleration in the body-frame (Ax, Ay, Az) (m/s^2) of the vehicle. The adopted frame is f.r.d
     */
    mavsdk::Offboard::AccelerationNed acceleration_;

    /**
     * @ingroup mocap
     * @brief Message to set the position (X-Y-Z) and attitude (roll, pitch and yaw) of the vehicle, where the body frame of the vehicle
     * is expressed according to the (f.r.d) reference frame and the inertial frame is expressed in NED frame. By default we initialize
     * the covariancle of the position and attitude states to NAN, so that the EKF used by the onboard controller uses the internal parameters
     * for computing the covariance. Moreover, the time is not specified such that the delay assumed between communications is also the one
     * defined by the internal parameters of the microcontroller.
     */
    mavsdk::Mocap::VisionPositionEstimate mocap_pose_;
};