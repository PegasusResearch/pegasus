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
#include <Eigen/Dense>
#include "ros_node.hpp"
#include "mavlink_node.hpp"
#include "pegasus_utils/frames.hpp"
#include "pegasus_utils/rotations.hpp"

/**
 * @brief Constructor for the ROS2 node that receives control commands 
 * from ROS and send sensor data to ROS2
 */
ROSNode::ROSNode() : rclcpp::Node("mavlink_node") {}

/**
 * @brief Destroy the ROSNode object
 */
ROSNode::~ROSNode() {}

/**
 * @brief Method used to start the ROS2 node in one thread and mavsdk in another thread
 */
void ROSNode::start() {

    // Initialize the parameters
    init_parameters();

    // Attemp to initialize the thrustcurve object such that a controller can specify 
    // to the driver the total thrust to apply to the vehicle in (N) and the conversion is made implicitly
    // to a percentage of the maximum thrust that the vehicle is capable of outputing [0-100%]
    try{
        init_thrust_curve();
    } catch(const std::runtime_error &error) {
        RCLCPP_WARN_STREAM(this->get_logger(), error.what());
        RCLCPP_WARN_STREAM(this->get_logger(), "Could not initilize thrust curve. The mavlink driver will only be able to receive the desired thrust in percentage topics");
    }

    // Initialize the mavlink node in this thread
    RCLCPP_INFO(this->get_logger(), "Starting mavlink node");
    mavlink_node_ = std::make_unique<MavlinkNode>(this->mavlink_config_);

    // Add this node to the multithread executor
    executor_.add_node(this->shared_from_this());

    // Start the executor in a separate thread
    RCLCPP_INFO(this->get_logger(), "Starting ROS thread node");
    this->executor_.spin();
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 parameters that we are supposed to read
 * from the parameter server
 */
void ROSNode::init_parameters() {
    
    // Get the connection address for the vehicle onboard computer
    this->declare_parameter<std::string>("connection", "tcp://:14550");
    rclcpp::Parameter connection_address = this->get_parameter("connection");

    // Get the ips to forward the mavlink messages to (by default the vector of ips is empty)
    this->declare_parameter<std::vector<std::string>>("mavlink_forward", std::vector<std::string>());
    rclcpp::Parameter mavlink_forward_ips = this->get_parameter("mavlink_forward");

    // Get the rates at which to receive the telemetry data from the onboard micro-controller
    this->declare_parameter<double>("mavlink_interface.rates.attitude", 50.0);
    this->declare_parameter<double>("mavlink_interface.rates.position", 30.0);
    this->declare_parameter<double>("mavlink_interface.rates.gps", 0.0);
    this->declare_parameter<double>("mavlink_interface.rates.altitude", 0.0);
    this->declare_parameter<double>("mavlink_interface.rates.imu", 0.0);
    mavlink_config_.rate_attitude = this->get_parameter("mavlink_interface.rates.attitude").as_double();
    mavlink_config_.rate_position = this->get_parameter("mavlink_interface.rates.position").as_double();
    mavlink_config_.rate_gps = this->get_parameter("mavlink_interface.rates.gps").as_double();
    mavlink_config_.rate_altitude = this->get_parameter("mavlink_interface.rates.altitude").as_double();
    mavlink_config_.rate_imu = this->get_parameter("mavlink_interface.rates.imu").as_double();

    // Log the rates
    RCLCPP_INFO_STREAM(this->get_logger(), "Telemetry rate - attitude: " << mavlink_config_.rate_attitude);
    RCLCPP_INFO_STREAM(this->get_logger(), "Telemetry rate - position: " << mavlink_config_.rate_position);
    RCLCPP_INFO_STREAM(this->get_logger(), "Telemetry rate - gps: " << mavlink_config_.rate_gps);
    RCLCPP_INFO_STREAM(this->get_logger(), "Telemetry rate - altitude: " << mavlink_config_.rate_altitude);
    RCLCPP_INFO_STREAM(this->get_logger(), "Telemetry rate - imu: " << mavlink_config_.rate_imu);

    mavlink_config_.connection_address = connection_address.as_string();
    mavlink_config_.forward_ips = mavlink_forward_ips.as_string_array();
    mavlink_config_.on_discover_callback = std::bind(&ROSNode::update_system_id, this, std::placeholders::_1);
    mavlink_config_.on_initialize_telemetry_callback = std::bind(&ROSNode::init_publishers, this);
    mavlink_config_.on_initialize_actions_callback = std::bind(&ROSNode::init_subscribers_and_services, this);

    // Callbacks for handling the Telemetry raw sensor data received by the vehicle (IMU, barometer and gps)
    mavlink_config_.on_imu_callback = std::bind(&ROSNode::on_imu_callback, this, std::placeholders::_1);
    mavlink_config_.on_altitude_callback = std::bind(&ROSNode::on_altitude_callback, this, std::placeholders::_1);
    mavlink_config_.on_raw_gps_callback = std::bind(&ROSNode::on_raw_gps_callback, this, std::placeholders::_1);
    mavlink_config_.on_gps_info_callback = std::bind(&ROSNode::on_gps_info_callback, this, std::placeholders::_1);

    // Callbacks for the filtered state of the vehicle (attitude, position and velocity) provided by EKF2
    mavlink_config_.on_quaternion_callback = std::bind(&ROSNode::on_quaternion_callback, this, std::placeholders::_1);
    mavlink_config_.on_angular_velocity_callback = std::bind(&ROSNode::on_angular_velocity_callback, this, std::placeholders::_1);
    mavlink_config_.on_position_velocity_callback = std::bind(&ROSNode::on_position_velocity_callback, this, std::placeholders::_1);

    // Callbacks for handling the Status and operating modes of the vehicle
    mavlink_config_.on_armed_callback = std::bind(&ROSNode::on_armed_callback, this, std::placeholders::_1);
    mavlink_config_.on_landed_state_callback = std::bind(&ROSNode::on_landed_state_callback, this, std::placeholders::_1);
    mavlink_config_.on_flight_mode_callback = std::bind(&ROSNode::on_flight_mode_callback, this, std::placeholders::_1);
    mavlink_config_.on_health_callback = std::bind(&ROSNode::on_health_callback, this, std::placeholders::_1);
    mavlink_config_.on_battery_callback = std::bind(&ROSNode::on_battery_callback, this, std::placeholders::_1);
    mavlink_config_.on_rc_callback = std::bind(&ROSNode::on_rc_callback, this, std::placeholders::_1);
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 publishers
 */
void ROSNode::init_publishers() {
    
    // ------------------------------------------------------------------------
    // Initialize the publisher for the status of the vehicle (arm/disarm state, connection, ...)
    // ------------------------------------------------------------------------
    this->declare_parameter("publishers.status", "status");
    rclcpp::Parameter status_topic = this->get_parameter("publishers.status");
    status_pub_ = this->create_publisher<pegasus_msgs::msg::Status>(status_topic.as_string(), rclcpp::SensorDataQoS());

    this->declare_parameter("publishers.vehicle_constants", "vehicle_constants");
    rclcpp::Parameter vehicle_constants_topic = this->get_parameter("publishers.vehicle_constants");
    vehicle_constants_pub_ = this->create_publisher<pegasus_msgs::msg::VehicleConstants>(vehicle_constants_topic.as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for sensors data (IMU, barometer and gps)
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("publishers.sensors.imu", "sensors/imu");
    rclcpp::Parameter imu_vel_accel_topic = this->get_parameter("publishers.sensors.imu");
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_vel_accel_topic.as_string(), rclcpp::SensorDataQoS());

    this->declare_parameter<std::string>("publishers.sensors.barometer", "sensors/barometer");
    rclcpp::Parameter barometer_topic = this->get_parameter("publishers.sensors.barometer");
    baro_pub_ = this->create_publisher<pegasus_msgs::msg::SensorBarometer>(barometer_topic.as_string(), rclcpp::SensorDataQoS());

    this->declare_parameter<std::string>("publishers.sensors.gps", "sensors/gps");
    rclcpp::Parameter gps_topic = this->get_parameter("publishers.sensors.gps");
    gps_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGps>(gps_topic.as_string(), rclcpp::SensorDataQoS());

    this->declare_parameter<std::string>("publishers.sensors.gps_info", "sensors/gps_info");
    rclcpp::Parameter gps_info_topic = this->get_parameter("publishers.sensors.gps_info");
    gps_info_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGpsInfo>(gps_info_topic.as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for the current state of the vehicle 
    // (position, orientation, body and inertial frame velocity)
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("publishers.filter.state", "filter/state");
    rclcpp::Parameter state_topic = this->get_parameter("publishers.filter.state");
    filter_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(state_topic.as_string(), rclcpp::SensorDataQoS());

    this->declare_parameter<std::string>("publishers.filter.rpy", "filter/rpy");
    rclcpp::Parameter rpy_topic = this->get_parameter("publishers.filter.rpy");
    filter_state_rpy_pub_ = this->create_publisher<pegasus_msgs::msg::RPY>(rpy_topic.as_string(), rclcpp::SensorDataQoS());
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 subscribers
 */
void ROSNode::init_subscribers_and_services() {

    // ------------------------------------------------------------------------
    // Subscribe to the position control (north-east-down in meters NED) and desired yaw (in deg)
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("subscribers.control.position", "control/position"); 
    rclcpp::Parameter position_control_topic = this->get_parameter("subscribers.control.position");
    position_control_sub_ = this->create_subscription<pegasus_msgs::msg::ControlPosition>(
        position_control_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::position_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the attitude (roll, pitch, yaw NED frame) and desired total thrust (0-100%)
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("subscribers.control.thrust.attitude", "control/attitude_thrust");
    rclcpp::Parameter attitude_thrust_topic = this->get_parameter("subscribers.control.thrust.attitude");
    attitude_thrust_sub_ = this->create_subscription<pegasus_msgs::msg::ControlAttitude>(
        attitude_thrust_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::attitude_thrust_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the attitude rate (roll-rate, pitch-rate, yaw-rate in NED frame) and desired total thrust (0-100%)
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("subscribers.control.thrust.attitude_rate", "control/attitude_rate_thrust");
    rclcpp::Parameter attitude_rate_thrust_topic = this->get_parameter("subscribers.control.thrust.attitude_rate");
    attitude_rate_thrust_sub_ = this->create_subscription<pegasus_msgs::msg::ControlAttitude>(
        attitude_rate_thrust_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::attitude_rate_thrust_callback, this, std::placeholders::_1));
    
    // If there is a thrust curve object already available, make 2 subcribers for controlling the 
    // attitude or attitude_rate along with the total desired force along the Z-axis expressed in Newton (N)
    // such that someone implementing a controller in another ROS node does not have to worry about the 
    // conversition from Forces to other unit such as percentage
    if(thrust_curve_) {
        this->declare_parameter<std::string>("subscribers.control.force.attitude", "control/attitude_force");
        rclcpp::Parameter attitude_force_topic = this->get_parameter("subscribers.control.force.attitude");
        attitude_force_sub_ = this->create_subscription<pegasus_msgs::msg::ControlAttitude>(
            attitude_force_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::attitude_force_callback, this, std::placeholders::_1));
        
        this->declare_parameter<std::string>("subscribers.control.force.attitude_rate", "control/attitude_rate_force");
        rclcpp::Parameter attitude_rate_force_topic = this->get_parameter("subscribers.control.force.attitude_rate");
        attitude_rate_force_sub_ = this->create_subscription<pegasus_msgs::msg::ControlAttitude>(
            attitude_rate_force_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::attitude_rate_force_callback, this, std::placeholders::_1));
    }

    // ------------------------------------------------------------------------
    // Subscribe to data comming from the motion capture system (MOCAP)
    // ------------------------------------------------------------------------

    // Get the ROS vehicle id and namespace (use to get the vehicle from the mocap system)
    this->declare_parameter<int>("vehicle_id", 1);
    rclcpp::Parameter vehicle_id = this->get_parameter("vehicle_id");

    this->declare_parameter<std::string>("vehicle_ns", "drone");
    rclcpp::Parameter vehicle_ns = this->get_parameter("vehicle_ns");

    this->declare_parameter<std::string>("subscribers.external_sensors.mocap_enu", "/mocap/pose_enu");
    rclcpp::Parameter mocap_topic = this->get_parameter("subscribers.external_sensors.mocap_enu");
    mocap_pose_enu_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_topic.as_string() + "/" + vehicle_ns.as_string() + std::to_string(vehicle_id.as_int()), 
        rclcpp::SensorDataQoS(), std::bind(&ROSNode::mocap_pose_callback, this, std::placeholders::_1));


    // ------------------------------------------------------------------------
    // Initiate the service to ARM/Disarm the vehicle
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("services.arm", "arm");
    rclcpp::Parameter arm_topic = this->get_parameter("services.arm");
    arm_service_ = this->create_service<pegasus_msgs::srv::Arm>(
        arm_topic.as_string(), std::bind(&ROSNode::arm_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // ------------------------------------------------------------------------
    // Initiate the service to kill switch the vehicle
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("services.kill_switch", "kill_switch");
    rclcpp::Parameter kill_switch_topic = this->get_parameter("services.kill_switch");
    kill_switch_service_ = this->create_service<pegasus_msgs::srv::KillSwitch>(
        kill_switch_topic.as_string(), std::bind(&ROSNode::kill_switch_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to Land the vehicle
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("services.land", "land");
    rclcpp::Parameter land_topic = this->get_parameter("services.land");
    land_service_ = this->create_service<pegasus_msgs::srv::Land>(
        land_topic.as_string(), std::bind(&ROSNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));


    // ------------------------------------------------------------------------
    // Initiate the service to set the vehicle mode
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("services.offboard", "offboard");
    rclcpp::Parameter offboard_topic = this->get_parameter("services.offboard");
    offboard_service_ = this->create_service<pegasus_msgs::srv::Offboard>(
        offboard_topic.as_string(), std::bind(&ROSNode::offboard_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter<std::string>("services.hold", "hold");
    rclcpp::Parameter position_hold_topic = this->get_parameter("services.hold");
    position_hold_service_ = this->create_service<pegasus_msgs::srv::PositionHold>(
        position_hold_topic.as_string(), std::bind(&ROSNode::position_hold_callback, this, std::placeholders::_1, std::placeholders::_2));


    // ------------------------------------------------------------------------
    // Initiate the service to set the home position
    // ------------------------------------------------------------------------
    this->declare_parameter<std::string>("services.set_home", "set_home");
    rclcpp::Parameter set_home_topic = this->get_parameter("services.set_home");
    set_home_position_service_ = this->create_service<pegasus_msgs::srv::SetHomePosition>(
        set_home_topic.as_string(), std::bind(&ROSNode::set_home_position_callback, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize the ThrustCurve object used
 * to translate the thrust curve from Newton (N) to percentage (0-100%) and vice-versa
 * based on the configurations loaded from ROS parameter server
 */
void ROSNode::init_thrust_curve() {
    
    // Get the singleton factory of thrust curves
    auto thrust_curve_Factory = Pegasus::ThrustCurveFactory::get_instance();

    // Get from the ROS parameter server the mass of the vehicle
    this->declare_parameter<double>("dynamics.mass", 0.0);
    rclcpp::Parameter mass = this->get_parameter("dynamics.mass");

    // Get from the ROS parameter server the type of thrust curve to apply to the vehicle
    this->declare_parameter<std::string>("dynamics.thrust_curve.identifier", "None"); 
    rclcpp::Parameter thrust_curve_id = this->get_parameter("dynamics.thrust_curve.identifier");

    // Get from the ROS parameter server the parameters that define the chosen thrust curve
    this->declare_parameter<std::vector<std::string>>("dynamics.thrust_curve.parameter_names", std::vector<std::string>()); 
    rclcpp::Parameter thrust_curve_parameter_names = this->get_parameter("dynamics.thrust_curve.parameter_names");

    this->declare_parameter<std::vector<double>>("dynamics.thrust_curve.parameters", std::vector<double>());
    rclcpp::Parameter thrust_curve_parameters = this->get_parameter("dynamics.thrust_curve.parameters");

    // Set the message with the parameters of the drone, namely the mass and the thrust curve
    vehicle_constants_msg_.mass = mass.as_double();
    vehicle_constants_msg_.thrust_curve.identifier = thrust_curve_id.as_string();
    vehicle_constants_msg_.thrust_curve.parameters = thrust_curve_parameter_names.as_string_array();
    vehicle_constants_msg_.thrust_curve.values = thrust_curve_parameters.as_double_array();

    // Check if the thrust curve parameter_names has the same size as the number of parameters - if not, throw a runtime error
    std::vector<double> parameters = thrust_curve_parameters.as_double_array();
    std::vector<std::string> parameter_names = thrust_curve_parameter_names.as_string_array();
    
    if(parameters.size() != parameter_names.size()) {
        throw std::runtime_error("Configuration for the thrust curve parameters and parameters names have a diferent sizes");
    }

    // Create a map of parameters to generate a thrust curve object
    std::map<std::string, double> gains;

    for(unsigned int i = 0; i < parameters.size(); i++ ) {
        gains[parameter_names[i]] = parameters[i];
    }

    // Instantiate a thrust curve object
    thrust_curve_ = thrust_curve_Factory.create_thrust_curve(gains, thrust_curve_id.as_string());
}

/**
 * @ingroup initFunctions
 * @brief Method that is called to update the system_id field in the status_msg. This method
 * DOES NOT publish the most up to date message to the status_pub
 * @param id An unsigned int (a byte) that encodes the mavlink id of the connected vehicle
 */
void ROSNode::update_system_id(const uint8_t & id) {
    status_msg_.system_id = id;
}

/**
 * @defgroup subscriberCallbacks
 * This group defines all the ROS subscriber callbacks
 */

/**
 * @ingroup subscriberCallbacks
 * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
 * @param msg A message with the desired position for the vehicle in NED
 */
void ROSNode::position_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg) {
    // Send the position reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_position(msg->position[0], msg->position[1], msg->position[2], msg->yaw);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude and thrust to apply to the vehicle
 */
void ROSNode::attitude_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    // Send the attitude and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude(msg->attitude[0], msg->attitude[1], msg->attitude[2], msg->thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
 */
void ROSNode::attitude_rate_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude_rate(msg->attitude[0], msg->attitude[1], msg->attitude[2], msg->thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude and force to apply to the vehicle
 */
void ROSNode::attitude_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {

    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude(msg->attitude[0], msg->attitude[1], msg->attitude[2], thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude-rate and force to apply to the vehicle
 */
void ROSNode::attitude_rate_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    
    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    // Log the thrust in percentage
    //RCLCPP_WARN_STREAM(this->get_logger(), "Thrust in percentage: " << thrust);

    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude_rate(msg->attitude[0], msg->attitude[1], msg->attitude[2], thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via mavlink to the vehicle autopilot filter to merge
 * @param msg A message with the pose of the vehicle expressed in ENU
 */
void ROSNode::mocap_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    
    // Convert the position expressed in ENU {East-North-Up} to NED {North-East-Down}
    Eigen::Vector3d position_ned = Pegasus::Frames::transform_vect_inertial_enu_ned(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

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

    // Send the mocap measured vehicle pose thorugh mavlink for the onboard microcontroller
    // to fuse in its internal EKF
    mavlink_node_->update_mocap_telemetry(position_ned, orientation_frd_ned);
}

/**
 * @defgroup publisherMessageUpdate
 * This group defines all the methods used to update the messages which encode the status/state of the vehicle
 * and that will be published to ROS2 via a publisher
 */

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the imu_msg. This method publishes
 * the most up to date message to imu_pub.
 * @param imu A mavsdk structure which contains:
 * - the angular velocity measured by the imu (in rad)
 * - the linear acceleration measured by the imu (in m/s^2)
 */
void ROSNode::on_imu_callback(const mavsdk::Telemetry::Imu &imu) {

    // Set the current timestamp
    imu_msg_.header.stamp = rclcpp::Clock().now();

    // Angular velocity measured directly by the IMU
    imu_msg_.angular_velocity.x = imu.angular_velocity_frd.forward_rad_s;
    imu_msg_.angular_velocity.y = imu.angular_velocity_frd.right_rad_s;
    imu_msg_.angular_velocity.z = imu.angular_velocity_frd.down_rad_s;

    // Linear acceleration measured directly by the IMU
    imu_msg_.linear_acceleration.x = imu.acceleration_frd.forward_m_s2;
    imu_msg_.linear_acceleration.y = imu.acceleration_frd.right_m_s2;
    imu_msg_.linear_acceleration.z = imu.acceleration_frd.down_m_s2;
    imu_pub_->publish(imu_msg_);
}

void ROSNode::on_altitude_callback(const mavsdk::Telemetry::Altitude & altitude) {
    
    // Set the current timestamp
    baro_msg_.header.stamp = rclcpp::Clock().now();

    // Set the altitude fields in meters
    baro_msg_.altitude_monotonic = altitude.altitude_monotonic_m;
    baro_msg_.altitude_amsl = altitude.altitude_amsl_m;
    baro_msg_.altitude_local = altitude.altitude_local_m;
    baro_msg_.altitude_relative_home = altitude.altitude_relative_m;
    baro_msg_.altitude_relative_terrain = altitude.altitude_terrain_m;
    baro_msg_.bottom_clearance = altitude.bottom_clearance_m;

    // Publish the altitude message
    baro_pub_->publish(baro_msg_);
}

void ROSNode::on_raw_gps_callback(const mavsdk::Telemetry::RawGps & gps) {
    
    // Set the current timestamp
    gps_msg_.header.stamp = rclcpp::Clock().now();

    // Set the GPS fields
    gps_msg_.latitude_deg = gps.latitude_deg;
    gps_msg_.longitude_deg = gps.longitude_deg;
    gps_msg_.altitude_msl = gps.absolute_altitude_m;
    gps_msg_.altitude_ellipsoid = gps.altitude_ellipsoid_m;
    gps_msg_.hdop = gps.hdop;
    gps_msg_.vdop = gps.vdop;
    gps_msg_.velocity = gps.velocity_m_s;
    gps_msg_.heading = gps.yaw_deg;
    gps_msg_.cog_deg = gps.cog_deg;
    gps_msg_.horizontal_uncertainty = gps.horizontal_uncertainty_m;
    gps_msg_.vertical_uncertainty = gps.vertical_uncertainty_m;
    gps_msg_.velocity_uncertainty = gps.velocity_uncertainty_m_s;
    gps_msg_.heading_uncertainty = gps.heading_uncertainty_deg;

    // Publish the GPS message
    gps_pub_->publish(gps_msg_);
}

void ROSNode::on_gps_info_callback(const mavsdk::Telemetry::GpsInfo & gps_info) {

    // Set the current timestamp
    gps_info_msg_.header.stamp = rclcpp::Clock().now();

    // Set the GPS info fields
    gps_info_msg_.num_satellites = gps_info.num_satellites;
    gps_info_msg_.fix_type = static_cast<uint8_t>(gps_info.fix_type);

    // Publish the GPS info message
    gps_info_pub_->publish(gps_info_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the pose.orientation field in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param quat A mavsdk structure which contains a quaternion encoding the attitude of the vehicle in NED
 */
void ROSNode::on_quaternion_callback(const mavsdk::Telemetry::Quaternion &quat) {

    // Set the current timestamp
    filter_state_msg_.header.stamp = rclcpp::Clock().now();

    // Set the attitude fields
    filter_state_msg_.pose.pose.orientation.w = quat.w;
    filter_state_msg_.pose.pose.orientation.x = quat.x;
    filter_state_msg_.pose.pose.orientation.y = quat.y;
    filter_state_msg_.pose.pose.orientation.z = quat.z;

    // Create the Eigen quaternion object and convert the angle to roll, pitch and yaw
    Eigen::Vector3d euler_angles = Pegasus::Rotations::quaternion_to_euler(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));

    // Fill in the RPY message
    filter_state_rpy_msg_.header.stamp = filter_state_msg_.header.stamp;
    filter_state_rpy_msg_.roll = Pegasus::Rotations::rad_to_deg(euler_angles(0));
    filter_state_rpy_msg_.pitch = Pegasus::Rotations::rad_to_deg(euler_angles(1));
    filter_state_rpy_msg_.yaw = Pegasus::Rotations::rad_to_deg(euler_angles(2));

    // Publish the updated message
    filter_state_pub_->publish(filter_state_msg_);

    // Publish the euler angles
    filter_state_rpy_pub_->publish(filter_state_rpy_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the body_vel.twist field in the state_msg. This method 
 * publishes the most up to date message to state_pub
 * @param ang_vel A mavsdk structure which contains the angular velocity of the vehicle expressed in the body frame
 * according to the f.r.d frame
 */
void ROSNode::on_angular_velocity_callback(const mavsdk::Telemetry::AngularVelocityBody &ang_vel) {

    // Set the current timestamp
    filter_state_msg_.header.stamp = rclcpp::Clock().now();

    // Set the angular velocity fields
    filter_state_msg_.twist.twist.angular.x = Pegasus::Rotations::rad_to_deg(ang_vel.roll_rad_s);
    filter_state_msg_.twist.twist.angular.y = Pegasus::Rotations::rad_to_deg(ang_vel.pitch_rad_s);
    filter_state_msg_.twist.twist.angular.z = Pegasus::Rotations::rad_to_deg(ang_vel.yaw_rad_s);

    // Publish the updated message
    filter_state_pub_->publish(filter_state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the pose and inertial_vel fields in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param pos_vel_ned A mavsdk structure which contains the position and linear velocity of the vehicle expressed in the inertial frame
 * in NED
 */
void ROSNode::on_position_velocity_callback(const mavsdk::Telemetry::PositionVelocityNed &pos_vel_ned) {
    
    // Set the current timestamp
    filter_state_msg_.header.stamp = rclcpp::Clock().now();

    // Set the position fields
    filter_state_msg_.pose.pose.position.x = pos_vel_ned.position.north_m;
    filter_state_msg_.pose.pose.position.y = pos_vel_ned.position.east_m;
    filter_state_msg_.pose.pose.position.z = pos_vel_ned.position.down_m;

    // Set the linear inertial velocity fields
    filter_state_msg_.twist.twist.linear.x = pos_vel_ned.velocity.north_m_s;
    filter_state_msg_.twist.twist.linear.y = pos_vel_ned.velocity.east_m_s;
    filter_state_msg_.twist.twist.linear.z = pos_vel_ned.velocity.down_m_s;
    
    // Publish the updated message
    filter_state_pub_->publish(filter_state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the armed field in the status_msg. This method
 * publishes the most up to date message to the status_pub
 * @param is_armed A boolean whether the vehicle is armed(1)/disarmed(0)
 */
void ROSNode::on_armed_callback(const bool &is_armed) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();

    // Set the armed field
    status_msg_.armed = is_armed;

    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the landed_state field in the status_msg. It is an enum, which
 * can assume the following values:
 * - 0 - UKNOWN
 * - 1 - ON_GROUND
 * - 2 - IN_AIR
 * - 3 - TAKING_OFF
 * - 4 - LANDING
 * @param landed_state A mavsdk structure wich contains the high level state of the vehicle
 */
void ROSNode::on_landed_state_callback(const mavsdk::Telemetry::LandedState & landed_state) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();

    // Set the landed_state field
    status_msg_.landed_state = static_cast<uint8_t>(landed_state);

    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the flight_mode field in the status_msg. This method
 * publishes the most up to date message to the status_pub
 * @param flight_mode An byte that containts the current flightmode active in the onboard microcontroller
 */
void ROSNode::on_flight_mode_callback(const mavsdk::Telemetry::FlightMode & flight_mode) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();

    // Set the current flight mode
    status_msg_.flight_mode = static_cast<uint8_t>(flight_mode);

    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the health field in the status_msg. Contains the variables:
 * - is_armable
 * - accelerometer_calibrated
 * - magnetometer_calibrated
 * - local_position_ok
 * - global_position_ok
 * - home_position_ok
 * This method publishes the most up to date message to status_pub
 * @param health A mavsdk structure which containts the state of the above vehicle variables
 */
void ROSNode::on_health_callback(const mavsdk::Telemetry::Health &health) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();

    // Set the health fields
    status_msg_.health.is_armable = health.is_armable;
    status_msg_.health.accelerometer_calibrated = health.is_accelerometer_calibration_ok;
    status_msg_.health.magnetometer_calibrated = health.is_magnetometer_calibration_ok;

    status_msg_.health.local_position_ok = health.is_local_position_ok;
    status_msg_.health.global_position_ok = health.is_global_position_ok;
    status_msg_.health.home_position_ok = health.is_home_position_ok;

    // Also publish the message with the vehicle constants
    // TODO: improve this later on
    vehicle_constants_pub_->publish(vehicle_constants_msg_);
    
    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the remaining battery percentage field in the status_msg. This method
 * publishes the most up to date message to status_pub
 */
void ROSNode::on_battery_callback(const mavsdk::Telemetry::Battery & battery) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();
    
    // Set the battery percentage field
    status_msg_.battery.id = battery.id;
    status_msg_.battery.temperature = battery.temperature_degc;
    status_msg_.battery.voltage = battery.voltage_v;
    status_msg_.battery.percentage = battery.remaining_percent;
    status_msg_.battery.current = battery.current_battery_a;
    status_msg_.battery.amps_hour_consumed = battery.capacity_consumed_ah;

    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the signal strength of a connected vehicle RC controller field in the status_msg. This method
 * publishes the most up to date message to status_pub
 * @param rc_signal A signal from the RC remote
 */
void ROSNode::on_rc_callback(const mavsdk::Telemetry::RcStatus & rc_signal) {

    // Set the current timestamp
    status_msg_.header.stamp = rclcpp::Clock().now();
    
    // Set the RC signal strength field
    status_msg_.rc_status.available = rc_signal.is_available;
    status_msg_.rc_status.signal_strength = rc_signal.signal_strength_percent * 100.0;

    // Publish the updated message
    status_pub_->publish(status_msg_);
}

/**
 * @defgroup servicesCallbacks
 * This group defines all the service server callbacks,
 * such as arming/disarming the vehicle or auto-landing
 */

/**
 * @ingroup servicesCallbacks
 * @brief Arming/disarming service callback. When a service request is reached from the arm_service_, 
 * this callback is called and will send a mavlink command for the vehicle to arm/disarm
 * @param request The request for arming (bool = true) or disarming (bool = false)
 * @param response The response in this service uint8
 */
void ROSNode::arm_callback(const pegasus_msgs::srv::Arm::Request::SharedPtr request, const pegasus_msgs::srv::Arm::Response::SharedPtr response) {
    // Set the response to the arm/disarm command
    response->success = mavlink_node_->arm_disarm(request->arm);
}

/**
 * @ingroup servicesCallbacks
 * @brief Kill switch service callback. When a service request is reached from the kill_switch_service_,
 * this callback is called and will send a mavlink command for the vehicle to kill the motors instantly.
 * @param request The request for killing the motors (bool = true)
 * @param response The response in this service uint8
*/
void ROSNode::kill_switch_callback(const pegasus_msgs::srv::KillSwitch::Request::SharedPtr request, const pegasus_msgs::srv::KillSwitch::Response::SharedPtr response) {
    
    // Set the response to the kill switch command
    response->success = request->kill == true ? mavlink_node_->kill_switch() : 0;
}

/**
 * @ingroup servicesCallbacks
 * @brief Autoland service callback. When a service request is reached from the land_service_,
 * this callback is called and will send a mavlink command for the vehicle to autoland using the onboard controller
 * @param request An empty request for landing the vehicle (can be ignored)
 * @param response The response in this service uint8
 */
void ROSNode::land_callback(const pegasus_msgs::srv::Land::Request::SharedPtr, const pegasus_msgs::srv::Land::Response::SharedPtr response) {

    // Set the response to the land command
    response->success = mavlink_node_->land();
}

/**
 * @ingroup servicesCallbacks
 * @brief Offboard service callback. When a service request is reached from the offboard_service_,
 * this callback is called and will send a mavlink command for the vehicle to enter offboard mode
 * @param request An empty request for entering offboard mode (can be ignored)
 * @param response The response in this service uint8
 */
void ROSNode::offboard_callback(const pegasus_msgs::srv::Offboard::Request::SharedPtr, const pegasus_msgs::srv::Offboard::Response::SharedPtr response) {

    // Set the response to the result of the offboard command
    response->success = mavlink_node_->offboard();
}

/**
 * @ingroup servicesCallbacks
 * @brief Position hold service callback. When a service request is reached from the position_hold_service_,
 * this callback is called and will send a mavlink command for the vehicle to enter position hold mode
 * @param request An empty request for entering position hold mode (can be ignored)
 * @param response The response in this service uint8
 */
void ROSNode::position_hold_callback(const pegasus_msgs::srv::PositionHold::Request::SharedPtr, const pegasus_msgs::srv::PositionHold::Response::SharedPtr response) {

    // Set the response to the result of the offboard command
    response->success = mavlink_node_->position_hold();
}

/**
 * @ingroup servicesCallbacks
 * @brief Set the home position callback. When a service request is reached from the set_home_position_service_,
 * this callback is called and will send a mavlink command for the vehicle to set the home position to the specified latitude, longitude and altitude
 * 
 * @param request The latitude, longitude and altitude of the home position
 * @param response None
 */
void ROSNode::set_home_position_callback(const pegasus_msgs::srv::SetHomePosition::Request::SharedPtr request, const pegasus_msgs::srv::SetHomePosition::Response::SharedPtr response) {
    // Call the mavlink node to set the home position
    mavlink_node_->set_home_position();
}
