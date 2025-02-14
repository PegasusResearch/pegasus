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
#include "mavlink_node.hpp"

/**
 * @brief Construct a new Mavlink Node object
 * @param config The configuration of the mavlink node
 */
MavlinkNode::MavlinkNode(const MavlinkNodeConfig &config) : config_(config) {

    // Add the connection using mavsdk with message forwarding, such that any message received/sent through this
    // address is shared among other connection addresses
    RCLCPP_INFO(rclcpp::get_logger("mavlink"), "Waiting to discover system on %s", config_.connection_address.c_str());
    mavsdk_.add_any_connection(config_.connection_address, mavsdk::ForwardingOption::ForwardingOn);

    // Define the callback that should be called when a new autopilot system is detected in the specified address
    new_system_handle_ = mavsdk_.subscribe_on_new_system(std::bind(&MavlinkNode::new_mavlink_system_callback, this));
}

/**
 * @brief Destroy the Mavlink Node object
 */
MavlinkNode::~MavlinkNode() {}

/**
 * @defgroup system_initializations
 * This group defines all the public control api callbacks for the vehicle
 */

/**
 * @ingroup system_initializations
 * @brief Method that is called whenever a new system is detected in the specified address by the mavsdk
 */

/**
 * @brief Method that is called whenever a new system is detected in the specified address by the mavsdk
 */
void MavlinkNode::new_mavlink_system_callback() {
    
    // Initialize the new detected system in another thread without blocking the mavsdk main thread loop
    std::thread([this] () {

        RCLCPP_INFO(rclcpp::get_logger("mavlink"), "Discovered system");

        // If the last received system does not have an autopilot, 
        // then just return and wait for a new system with an autopilot
        if (!this->mavsdk_.systems().back()->has_autopilot()) return;

        // Go through the systems list and get the latest connected system
        this->system_ = this->mavsdk_.systems().back();

        // Unsubscribe again as we only want to have one connected vehicle per ROS2 driver node
        this->mavsdk_.unsubscribe_on_new_system(this->new_system_handle_);

        // Update the Status message with the drone ID
        this->system_id_ = system_->get_system_id();

        // Set the configurations to use along with mavsdk (setup for this system to be a companion computer)
        configuration_ = std::make_shared<mavsdk::Mavsdk::Configuration>(mavsdk::Mavsdk::ComponentType::CompanionComputer);
        configuration_->set_always_send_heartbeats(false);
        
        // Update the configuration settings in the mavsdk object
        mavsdk_.set_configuration(*(configuration_.get()));

        // Inform the ROS side that a new connection was established
        this->config_.on_discover_callback(system_id_);

        // -----------------------------------------------
        // Instantiate the mavsdk plugins for this vehicle
        // -----------------------------------------------

        // (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates
        this->initialize_telemetry();

        // Send mocap data to the autopilot to fuse inside the onboard filter
        this->initialize_mocap();

        // Enable simple actions such as arming and landing
        this->initialize_actions();

        // Enable the mavlink pass-through to send and receive mavlink messages
        //this->initialize_mavlink_passthrough();

        // Control a drone with position, velocity, attitude or motor commands
        this->initialize_offboard();

        // Enable forwarding to other mavlink interfaces in the network (QGroundControl)
        this->initialize_mavlink_forwarding();

    }).detach();
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize all
 * the telemetry mavlink subscriptions and subsequent ROS2 publishers for vehicle data/state/status
 */
void MavlinkNode::initialize_telemetry() {

    // -----------------------------------------------------------------
    // Initialize the mavsdk telemetry module to receive battery status, GPS, RC connection, flight mode, vehicle state
    // and set the corresponding telemetry rates
    // -----------------------------------------------------------------
    telemetry_ = std::make_unique<mavsdk::Telemetry>(this->system_);

    // -----------------------------------------------------------------
    // Initialize all the necessary ROS2 publishers, before we receive any telemetry data from mavlink
    // -----------------------------------------------------------------
    config_.on_initialize_telemetry_callback();
    
    // ------------------------------------------------------------------
    // Initialize all the telemetry callbacks from mavsdk such that each calls the corresponding ros publisher
    // ------------------------------------------------------------------

    // Subscribe to the raw data from the imu, barometer and gps
    this->telemetry_->subscribe_imu(config_.on_imu_callback);
    this->telemetry_->subscribe_altitude(config_.on_altitude_callback);
    this->telemetry_->subscribe_raw_gps(config_.on_raw_gps_callback);
    this->telemetry_->subscribe_gps_info(config_.on_gps_info_callback);
    this->telemetry_->subscribe_distance_sensor(config_.on_distance_sensor_callback);

    // Subscribe to the filtered pose of the vehicle (given by the EKF2 filter + complementary filter)
    this->telemetry_->subscribe_position_velocity_ned(config_.on_position_velocity_callback);
    this->telemetry_->subscribe_attitude_quaternion(config_.on_quaternion_callback);
    this->telemetry_->subscribe_attitude_angular_velocity_body(config_.on_angular_velocity_callback);
    
    // Subscribe to the status and operating modes of the vehicle
    this->telemetry_->subscribe_armed(config_.on_armed_callback);
    this->telemetry_->subscribe_landed_state(config_.on_landed_state_callback);
    this->telemetry_->subscribe_flight_mode(config_.on_flight_mode_callback);
    this->telemetry_->subscribe_health(config_.on_health_callback);
    this->telemetry_->subscribe_battery(config_.on_battery_callback);
    this->telemetry_->subscribe_rc_status(config_.on_rc_callback);

    // Set the rates at which to receive the telemetry data
    this->telemetry_->set_rate_attitude_quaternion(config_.rate_attitude);
    this->telemetry_->set_rate_position_velocity_ned(config_.rate_position);
    this->telemetry_->set_rate_gps_info(config_.rate_gps);
    this->telemetry_->set_rate_altitude(config_.rate_altitude);
    this->telemetry_->set_rate_imu(config_.rate_imu);
    this->telemetry_->set_rate_distance_sensor(config_.rate_distance);
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize all
 * the actions that we can send through mavlink and create the corresponding ROS2 actions for arming/disarming and auto-landing
 */
void MavlinkNode::initialize_actions() {

    // Initialize the mavsdk actions module to arm/disarm the vehicle and to auto-land the vehicle
    action_ = std::make_unique<mavsdk::Action>(this->system_);
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize the
 * mavlink passthrough submodule and allow for sending and receiving mavlink messages to and from the vehicle.
 */
// void MavlinkNode::initialize_mavlink_passthrough() {

//     // Initialize the mavsdk pass-through module to send and receive mavlink messages
//     mavlink_passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(this->system_);
// }

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detect to initialize all
 * the offboard mode controllers that can be used via mavlink and create the corresponding ROS2 subscribers
 * for receiving such control inputs
 */
void MavlinkNode::initialize_offboard() {

    // -----------------------------------------------------------------
    // Initialize the mavsdk offboard module to control a drone 
    // with position, velocity, attitude or motor commands
    // -----------------------------------------------------------------
    offboard_ = std::make_unique<mavsdk::Offboard>(this->system_);

    // ----------------------------------------------
    // Initialize all the necessary ROS2 subscribers
    // ----------------------------------------------
    config_.on_initialize_actions_callback();
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detect to initialize the
 * mocap mavsdk submodule and allow for publishing mocap data to be fused by the vehicle's onboard EFK filter
 */
void MavlinkNode::initialize_mocap() {

    mocap_ = std::make_unique<mavsdk::Mocap>(this->system_);

    // By default we initialize the covariancle of the position and attitude states to NAN, so that the EKF 
    // used by the onboard controller uses the internal parameters for computing the covariance. Moreover, 
    // the time is not specified such that the delay assumed between communications is also the one
    // defined by the internal parameters of the microcontroller.
    mocap_pose_.pose_covariance.covariance_matrix = std::vector<float>(21, 0.0);
    mocap_pose_.time_usec = 0;
    mocap_pose_.pose_covariance.covariance_matrix[0] = NAN;
    mocap_pose_.pose_covariance.covariance_matrix[15] = NAN;
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize
 * all mavlink redirections (like mavlink router) to the ips speficied in "mavlink_forward" ROS parameter
 */
void MavlinkNode::initialize_mavlink_forwarding() {
    
    // Iterate over the vector of ips and try to connect to them
    for(auto & ip : config_.forward_ips) {
        // Check if the string was empty, just skip
        if(ip.compare("") != 0 && ip.compare("\n") != 0) {
            mavsdk::ConnectionResult result = mavsdk_.add_any_connection(ip, mavsdk::ForwardingOption::ForwardingOn);

            // Inform the user that mavlink messages are now being forward to the given ip
            if(result == mavsdk::ConnectionResult::Success) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("mavlink"), "Mavlink messages forward to: " + ip);
            } else {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("mavlink"), "Mavlink messages failed to forward: " + ip);
            }
        }
    }
}

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
void MavlinkNode::set_attitude(const float roll, const float pitch, const float yaw, const float thrust) {
    
    // Populate the mavsdk structure (with attitude - roll, pitch and yaw in deg, following Z-Y-X convention) and 
    // total thrust scaled between 0-100%, converted to 0<->1 and send it through mavlink
    attitude_.roll_deg = roll;
    attitude_.pitch_deg = pitch;
    attitude_.yaw_deg = yaw;
    
    // Make sure the thrust value that get's requested is normalized between 0 and 1
    attitude_.thrust_value = std::min(std::max(thrust / 100.0, 0.0), 1.0); 
    
    // Send the message to the onboard vehicle controller
    offboard_->set_attitude(attitude_);
}

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
void MavlinkNode::set_attitude_rate(const float roll_rate, const float pitch_rate, const float yaw_rate, const float thrust) {
    
    // Populate the mavsdk structure (with attitude rate - roll-rate, pitch-rate, yaw-rate in deg/sec, following the Z-Y-X convention) and
    // total thrust scaled between 0-100%, converted to 0<->1 and send it through mavlink
    attitude_rate_.roll_deg_s = roll_rate;
    attitude_rate_.pitch_deg_s = pitch_rate; 
    attitude_rate_.yaw_deg_s = yaw_rate;
    
    // Make sure the thrust value that get's requested is normalized between 0 and 1
    attitude_rate_.thrust_value = std::min(std::max(thrust / 100.0, 0.0), 1.0); 
    
    // Send the message to the onboard vehicle controller
    offboard_->set_attitude_rate(attitude_rate_);
}

/**
 * @ingroup control_callbacks
 * @brief Method to set the position (X-Y-Z) of the vehicle. The adopted frame is NED
 * @param x The North position in the inertial NED frame
 * @param y The East position in the inertial NED frame
 * @param z The Down position in the inertial NED frame
 * @param yaw The yaw angle in deg
 */
void MavlinkNode::set_position(const float x, const float y, const float z, const float yaw) {
    
    // Populate the MAVSDK structure (with position X, Y, Z in inertial NED frame -> north-east-down) and the respective yaw angle (expressed in deg)
    position_.north_m = x;
    position_.east_m = y;
    position_.down_m = z;
    position_.yaw_deg = yaw;

    // Send the message to the onboard vehicle controller
    offboard_->set_position_ned(position_);
}

/**
 * @ingroup control_callbacks
 * @brief Set the inertial velocity (Vx, Vy, Vz) (m/s) and yaw (deg) of the vehicle. The adopted frame is NED
 * @param vx The velocity in the North direction in the inertial NED frame
 * @param vy The velocity in the East direction in the inertial NED frame
 * @param vz The velocity in the Down direction in the inertial NED frame
 * @param yaw The yaw angle in deg
 */
void MavlinkNode::set_inertial_velocity(const float vx, const float vy, const float vz, const float yaw) {
        
    // Populate the MAVSDK structure (with velocity Vx, Vy, Vz in inertial NED frame -> north-east-down) and the respective yaw angle (expressed in deg)
    inertial_velocity_.north_m_s = vx;
    inertial_velocity_.east_m_s = vy;
    inertial_velocity_.down_m_s = vz;
    inertial_velocity_.yaw_deg = yaw;

    // Send the message to the onboard vehicle controller
    offboard_->set_velocity_ned(inertial_velocity_);
}

/**
 * @ingroup control_callbacks
 * @brief Set the body velocity (Vx, Vy, Vz) (m/s) and yaw-rate (deg/s) of the vehicle. The adopted frame is f.r.d
 * Note that with quadrotors, the roll and pitch of the body frame-are ignored, and the vehicle is actuated in a "cheated" body velocity
 * @param vx The velocity in the x direction in the body frame (projected for 0 roll)
 * @param vy The velocity in the y direction in the body frame (projected for 0 pitch)
 * @param vz The velocity in the z direction in the body frame
 * @param yaw_rate The angular velocity around the z-axis in the body frame
 */
void MavlinkNode::set_body_velocity(const float vx, const float vy, const float vz, const float yaw_rate) {

    // Populate the MAVSDK structure (with velocity Vx, Vy, Vz in body frame -> forward-right-down) and the respective yaw rate (expressed in deg/s)
    body_velocity_.forward_m_s = vx;
    body_velocity_.right_m_s = vy;
    body_velocity_.down_m_s = vz;
    body_velocity_.yawspeed_deg_s = yaw_rate;

    // Send the message to the onboard vehicle controller
    offboard_->set_velocity_body(body_velocity_);
}

/**
 * @ingroup control_callbacks
 * @brief Set the inertial acceleration (Ax, Ay, Az) (m/s^2) of the vehicle. The adopted frame is NED
 * @param ax The acceleration in the North direction in the inertial NED frame
 * @param ay The acceleration in the East direction in the inertial NED frame
 * @param az The acceleration in the Down direction in the inertial NED frame
 */
void MavlinkNode::set_inertial_acceleration(const float ax, const float ay, const float az) {

    // Populate the MAVSDK structure (with acceleration Ax, Ay, Az in inertial NED frame -> north-east-down)
    acceleration_.north_m_s2 = ax;
    acceleration_.east_m_s2 = ay;
    acceleration_.down_m_s2 = az;

    // Send the message to the onboard vehicle controller
    offboard_->set_acceleration_ned(acceleration_);
}

/**
 * @brief Method to arm or disarm the vehicle
 * @param arm_disarm The boolean that is 1 to arm the vehicle and 0 to disarm
 */
uint8_t MavlinkNode::arm_disarm(const bool arm_disarm) {
    return static_cast<uint8_t>((arm_disarm) ? action_->arm() : action_->disarm());
}

uint8_t MavlinkNode::kill_switch() {
    return static_cast<uint8_t>(action_->kill());
}

/**
* @brief Sends a signal to control a specific motor or actuator.
* 
* @param index Specifies the index of the motor/actuator, "Actuactor Set", to control.
*              This should be an integer representing the target gate
* @param value Specifies the value to set for the actuator
*              The value should be a float, between -1.0 (minimum) and 1.0 (maximum)
*/
uint8_t MavlinkNode::set_motors(const int32_t index, const float value) {
    return static_cast<uint8_t>(action_->set_actuator(index, value));
}

/**
 * @brief Method to autoland the vehicle using the onboard microntroller controller
 */
uint8_t MavlinkNode::land() {
    return static_cast<uint8_t>(action_->land());
}

/**
 * @brief Method to make the vehicle switch to the offboard mode. This function is blocking
 * until we have a result from the vehicle
 */
uint8_t MavlinkNode::offboard() {
    return static_cast<uint8_t>(offboard_->start());
}

/**
 * @brief Method to make the vehicle switch to the position hold mode. This function is blocking
 * until we have a result from the vehicle. 
 */
uint8_t MavlinkNode::position_hold() {
    return static_cast<uint8_t>(action_->hold());
}

/**
 * @brief Method to set the home position of the onboard micro-controller.
 */
// void MavlinkNode::set_home_position() {
//     mavsdk::MavlinkPassthrough::CommandLong ComLong;
//     ComLong.target_compid = mavlink_passthrough_->get_target_compid();
//     ComLong.target_sysid = mavlink_passthrough_->get_target_sysid();
//     ComLong.command = 48;           // SET_GPS_GLOBAL_ORIGIN 
//     ComLong.param1 = static_cast<uint32_t>(394136138);    // Latitude
//     ComLong.param2 = static_cast<uint32_t>(-91361315);    // Longitude
//     ComLong.param3 = static_cast<uint32_t>(58881);        // Altitude (MSL)
//     ComLong.param4 = NAN;
//     ComLong.param5 = NAN;
//     ComLong.param6 = NAN;
//     ComLong.param7 = NAN;
//     mavlink_passthrough_->send_command_long(ComLong);
// }

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
void MavlinkNode::update_mocap_telemetry(const Eigen::Vector3d &position, const Eigen::Vector3d &attitude) {

    // Set the position of the vehicle in the inertial frame (expressed in NED)
    mocap_pose_.position_body.x_m = position.x();
    mocap_pose_.position_body.y_m = position.y();
    mocap_pose_.position_body.z_m = position.z();

    // Set the orientation of the vehicle in the inertial frame (expressed in NED)
    // with body frame expressed in f.r.d
    mocap_pose_.angle_body.roll_rad = attitude.x();
    mocap_pose_.angle_body.pitch_rad = attitude.y();
    mocap_pose_.angle_body.yaw_rad = attitude.z();

    // Send the current position to the vehicle through mavlink
    this->mocap_->set_vision_position_estimate(mocap_pose_);
}