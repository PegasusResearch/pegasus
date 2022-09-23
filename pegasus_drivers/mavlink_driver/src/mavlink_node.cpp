#include "ros_node.hpp"
#include "mavlink_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"

/**
 * @brief Construct a new Mavlink Node object
 * @param nh The ROS2 nodehandler necessary to read parameters from the node parameter server
 * @param ros_node The ros_node object that will contain all the publishers and subscribers for ROS
 */
MavlinkNode::MavlinkNode(const rclcpp::Node::SharedPtr nh, const std::shared_ptr<ROSNode> ros_node) : nh_(nh), ros_node_(ros_node) {

    // Get the connection address for the vehicle onboard microcontroller from the ROS2 parameter server
    nh_->declare_parameter<std::string>("connection", "tcp://:14550");
    rclcpp::Parameter connection_address = nh_->get_parameter("connection");

    // Add the connection using mavsdk with message forwarding, such that any message received/sent through this
    // address is shared among other connection addresses
    mavsdk_.add_any_connection(connection_address.as_string(), mavsdk::ForwardingOption::ForwardingOn);

    // Define the callback that should be called when a new autopilot system is detected in the specified address
    mavsdk_.subscribe_on_new_system(std::bind(&MavlinkNode::new_mavlink_system_callback, this));
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
        // If the last received system does not have an autopilot, 
        // then just return and wait for a new system with an autopilot
        if (!this->mavsdk_.systems().back()->has_autopilot()) return;

        // Go through the systems list and get the latest connected system
        this->system_ = this->mavsdk_.systems().back();

        // Unsubscribe again as we only want to have one connected vehicle per ROS2 driver node
        this->mavsdk_.subscribe_on_new_system(nullptr);

        // Update the Status message with the drone ID
        this->ros_node_->update_system_id(system_->get_system_id());

        // Set the configurations to use along with mavsdk (setup for this system to be a companion computer)
        configuration_ = std::make_shared<mavsdk::Mavsdk::Configuration>(mavsdk::Mavsdk::Configuration::UsageType::CompanionComputer);
        
        // Set the mavsdk to always send hearbeats (if possible)
        configuration_->set_always_send_heartbeats(true);
        
        // Update the configuration settings in the mavsdk object
        mavsdk_.set_configuration(*(configuration_.get()));

        // -----------------------------------------------
        // Instantiate the mavsdk plugins for this vehicle
        // -----------------------------------------------

        // (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates
        this->initialize_telemetry();

        // Enable simple actions such as arming and landing
        this->initialize_actions();

        // Control a drone with position, velocity, attitude or motor commands
        this->initialize_offboard();

        // Send mocap data to the autopilot to fuse inside the onboard filter
        this->initialize_mocap();

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
    ros_node_->init_publishers();
    
    // ------------------------------------------------------------------
    // Initialize all the telemetry callbacks from mavsdk such that each calls the corresponding ros publisher
    // ------------------------------------------------------------------
    // Subscribe to the current arm state, flight mode, battery level, RC strength of the vehicle
    this->telemetry_->subscribe_armed(std::bind(&MavlinkNode::armed_state_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_flight_mode(std::bind(&MavlinkNode::flightmode_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_battery(std::bind(&MavlinkNode::battery_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_rc_status(std::bind(&MavlinkNode::rc_status_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_health(std::bind(&MavlinkNode::health_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_landed_state(std::bind(&MavlinkNode::landed_state_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_actuator_control_target(std::bind(&MavlinkNode::actuator_reference_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_actuator_output_status(std::bind(&MavlinkNode::actuator_output_callback, this, std::placeholders::_1));

    // Subscribe to the raw data from the imu
    this->telemetry_->subscribe_imu(std::bind(&MavlinkNode::imu_callback, this, std::placeholders::_1));

    // Subscribe to the position/velocity in the NED inertial frame
    this->telemetry_->subscribe_position_velocity_ned(std::bind(&MavlinkNode::position_velocity_ned_callback, this, std::placeholders::_1));

    // Subscribe to the filtered attitude of the vehicle
    this->telemetry_->subscribe_attitude_quaternion(std::bind(&MavlinkNode::attitude_ned_quaternion_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_attitude_euler(std::bind(&MavlinkNode::attitude_ned_euler_callback, this, std::placeholders::_1));
    this->telemetry_->subscribe_attitude_angular_velocity_body(std::bind(&MavlinkNode::angular_velocity_body_callback, this, std::placeholders::_1));
    
    // Subscribe to the odometry message of the vehicle 
    this->telemetry_->subscribe_odometry(std::bind(&MavlinkNode::odometry_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------
    // Set the mavlink telemetry rates for each type of message received
    // ------------------------------------------------------------------

    // Get the connection address for the vehicle onboard microcontroller from the ROS2 parameter server
    nh_->declare_parameter<double>("rates.mavlink_telemetry.imu", 10.0);
    rclcpp::Parameter imu_rate = nh_->get_parameter("rates.mavlink_telemetry.imu");
    this->telemetry_->set_rate_imu(imu_rate.as_double());

    nh_->declare_parameter<double>("rates.mavlink_telemetry.pos_vel", 10.0);
    rclcpp::Parameter pos_vel_rate = nh_->get_parameter("rates.mavlink_telemetry.pos_vel");
    this->telemetry_->set_rate_position_velocity_ned(pos_vel_rate.as_double());
    
    nh_->declare_parameter<double>("rates.mavlink_telemetry.attitude", 10.0);
    rclcpp::Parameter attitude_rate = nh_->get_parameter("rates.mavlink_telemetry.attitude");
    this->telemetry_->set_rate_attitude(attitude_rate.as_double());

    nh_->declare_parameter<double>("rates.mavlink_telemetry.odometry", 10.0);
    rclcpp::Parameter odometry_rate = nh_->get_parameter("rates.mavlink_telemetry.odometry");
    this->telemetry_->set_rate_odometry(odometry_rate.as_double());
}

/**
 * @ingroup system_initializations
 * @brief Method that is called by new_mavlink_system_callback whenever a new system is detected to initialize all
 * the actions that we can send through mavlink and create the corresponding ROS2 actions for arming/disarming and auto-landing
 */
void MavlinkNode::initialize_actions() {

    // Initialize the mavsdk actions module to arm/disarm the vehicle and to auto-land the vehicle
    action_ = std::make_unique<mavsdk::Action>(this->system_);
    
    // Initialize the ROS2 side of the actions and services
    ros_node_->init_actions();
    ros_node_->init_services();
}

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
    ros_node_->init_subscribers();
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
    
    // Get the ips to forward the mavlink messages to from ROS parameter server (by default the vector of ips is empty)
    nh_->declare_parameter<std::vector<std::string>>("mavlink_forward", std::vector<std::string>());
    rclcpp::Parameter mavlink_forward_ips = nh_->get_parameter("mavlink_forward");

    // Iterate over the vector of ips and try to connect to them
    for(auto & ip : mavlink_forward_ips.as_string_array()) {
        // Check if the string was empty, just skip
        if(ip.compare("") != 0 && ip.compare("\n") != 0) {
            mavsdk::ConnectionResult result = mavsdk_.add_any_connection(ip, mavsdk::ForwardingOption::ForwardingOn);

            // Inform the user that mavlink messages are now being forward to the given ip
            if(result == mavsdk::ConnectionResult::Success) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Mavlink messages forward to: " + ip);
            } else {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Mavlink messages failed to forward: " + ip);
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
 * @brief Method to set the body velocity of the vehicle expressed in f.r.d frame and yaw-rate
 * @param forward The forward body velocity in m/s
 * @param right The right body velocity in m/s
 * @param down The down body velocity in m/s
 * @param yaw_rate The yaw rate of the vehicle expressed in degres/s
 */
void MavlinkNode::set_body_velocity(const float forward, const float right, const float down, const float yaw_rate) {
    
    // Update the body velocity message fields
    body_velocity_and_yaw_rate_deg_.forward_m_s = forward;
    body_velocity_and_yaw_rate_deg_.right_m_s = right;
    body_velocity_and_yaw_rate_deg_.down_m_s = down;
    body_velocity_and_yaw_rate_deg_.yawspeed_deg_s = yaw_rate;

    // Send the message to the onboard vehicle controller
    offboard_->set_velocity_body(body_velocity_and_yaw_rate_deg_);

    // Check if the vehicle is already in offboard mode - if not, then switch to offboard mode 
    check_switch_offboard_mode();
}

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

    // Check if the vehicle is already in offboard mode - if not, then switch to offboard mode 
    check_switch_offboard_mode();
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

    // Check if the vehicle is already in offboard mode - if not, then switch to offboard mode 
    check_switch_offboard_mode();
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

    // Check if the vehicle is already in offboard mode - if not, then switch to offboard mode 
    check_switch_offboard_mode();
}

/**
 * @brief Method to arm or disarm the vehicle
 * @param arm_disarm The boolean that is 1 to arm the vehicle and 0 to disarm
 */
void MavlinkNode::arm_disarm(const bool arm_disarm) {
    (arm_disarm) ? action_->arm_async(nullptr) : action_->disarm_async(nullptr);
}

/**
 * @brief Method to autoland the vehicle using the onboard microntroller controller
 */
void MavlinkNode::land() {
    action_->land_async(nullptr);
}

/**
 * @brief Method to reboot the vehicle onboard microcontroller (NOT THE ONBOARD PC!)
 */
void MavlinkNode::reboot() {
    action_->reboot_async(nullptr);
}

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
void MavlinkNode::check_switch_offboard_mode() {

    // Check if are on offboard mode already, if not, then switch to it
    if(ros_node_->get_current_flight_mode() != ros_node_->get_offboard_code_mode() && !starting_offboard_) {
        this->starting_offboard_ = true;

        // Initiate a timer to check 3 seconds later and make theusing std::chrono_literals; starting offboard variable false
        using namespace std::chrono_literals;
        offboard_check_timer_ = nh_->create_wall_timer(20s, [this] () {
            starting_offboard_ = false;
            offboard_check_timer_->cancel();
        });

        // Start the offboard mode inside the microcontroller and signal the user of the result
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Requesting offboard mode");
        
        offboard_->start_async([](mavsdk::Offboard::Result result) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Offboard mode: " << result);
        });
    }
}

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
void MavlinkNode::armed_state_callback(bool is_armed) {
    ros_node_->update_armed_state(is_armed);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current engaged fligh mode of the vehicle
 * @param flight_mode A mavsdk structure that contains all possible flight modes
 */
void MavlinkNode::flightmode_callback(mavsdk::Telemetry::FlightMode flight_mode) {
    ros_node_->update_flightmode_state(static_cast<unsigned char>(flight_mode));
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current battery status of the vehicle
 * @param battery A mavsdk structure that contains the id of the battery, voltage and percentage
 */
void MavlinkNode::battery_callback(mavsdk::Telemetry::Battery battery) {
    ros_node_->update_battery_state(battery.remaining_percent * 100.0);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current state of the RC controller of the vehicle
 * @param rc_status A mavsdk structure that contains data whether an RC controller is paired with the vehicle
 * and the corresponding signal strength
 */
void MavlinkNode::rc_status_callback(mavsdk::Telemetry::RcStatus rc_status) {
    ros_node_->update_rc_state(rc_status.is_available ? rc_status.signal_strength_percent * 100.0 : -1);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current health of the vehicle and sensors
 * @param health A mavsdk structure that contains data regarding IMU calibration, local positon, global position
 * and home position definition state and ARMED/DISARMED state of the vehicle
 */
void MavlinkNode::health_callback(mavsdk::Telemetry::Health health) {
    ros_node_->update_heath_state(health);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current status of the vehicle regarding its operation. It
 * informs the user if the vehicle is landed, in air, landing, taking off or in an unknown state
 * @param landed_state A mavsdk structure that contains data regarding the high level operation mode of the vehicle
 */
void MavlinkNode::landed_state_callback(mavsdk::Telemetry::LandedState landed_state) {
    ros_node_->update_landed_state(landed_state);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the reference values (between -1 and 1) that are reaching
 * the ESCs (Eletronic Speed Controllers) - these can be for thrusters or gimbal
 * @param actuator_reference A mavsdk structure that contains the actuator group and a vector with the references given
 * to that group of actuators
 */
void MavlinkNode::actuator_reference_callback(mavsdk::Telemetry::ActuatorControlTarget actuator_reference) {
    (void) actuator_reference;
    // TODO - nothing for now, but in the future might be nice when flying outside to have this telemetry avaialable
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current output of the actuators.
 * @param actuator_output A mavsdk structure that contains the number of active actuators and a vector with the 
 * output of those actuators
 */
void MavlinkNode::actuator_output_callback(mavsdk::Telemetry::ActuatorOutputStatus actuator_output) {
    (void) actuator_output;
    // TODO - nothing for now, but in the future might be nice when flying outside to have this telemetry avaialable
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the current state of the imu
 * @param imu A mavsdk structure that containts data regarding the angular velocity, linear acceleration and
 * magnetic field (received in Gauss)
 */
void MavlinkNode::imu_callback(mavsdk::Telemetry::Imu imu) {
    ros_node_->update_imu_state(imu);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the position and velocity of the vehicle
 * expressed in the NED inertial frame of reference
 * @param pos_vel_ned A mavsdk structure that containts the position and linear velocity of the vehicle
 * expressed in the NED inertial frame of reference
 */
void MavlinkNode::position_velocity_ned_callback(mavsdk::Telemetry::PositionVelocityNed pos_vel_ned) {
    ros_node_->update_position_velocity_ned(pos_vel_ned);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the attitude of the vehicle body reference frame expressed in (f.r.d)
 * with respect to the inertial frame in NED in the inertial frame of reference, using quaternion
 * @param quat A mavsdk quaternion structure
 */
void MavlinkNode::attitude_ned_quaternion_callback(mavsdk::Telemetry::Quaternion quat) {
    ros_node_->update_attitude_quaternion_ned(quat);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the attitude of the vehicle body reference frame expressed in (f.r.d)
 * with respect to the inertial frame in NED in the inertial frame of reference using Euler angles, according to a Z-Y-X
 * convention
 * @param euler A mavsdk structure with the euler angles roll, pitch and yaw expressed in degrees
 */
void MavlinkNode::attitude_ned_euler_callback(mavsdk::Telemetry::EulerAngle euler) {
    ros_node_->update_attitude_ned_euler(euler);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the angular velocity of the vehicle body reference frame expressed in (f.r.d)
 * using a Z-Y-X convention
 * @param ang_vel A mavsdk structure with the angle rate roll-rate, pitch-rate and yaw-rate expressed in degrees/second
 */
void MavlinkNode::angular_velocity_body_callback(mavsdk::Telemetry::AngularVelocityBody ang_vel) {
    ros_node_->update_angular_velocity_body(ang_vel);
}

/**
 * @ingroup vehicle_state_callbacks
 * @brief Method that is called periodically to update the state of the vehicle expressed in the body reference frame (f.r.d)
 * @param odom A mavsdk odom structure for which we only care about the linear body velocity of the vehicle
 * expressed in the body frame of reference (f.r.d)
 */
void MavlinkNode::odometry_callback(mavsdk::Telemetry::Odometry odom) {
    ros_node_->update_odometry(odom);
}

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