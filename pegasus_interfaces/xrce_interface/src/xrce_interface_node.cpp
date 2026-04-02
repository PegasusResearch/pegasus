/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
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
#include <limits>
#include <Eigen/Dense>
#include "xrce_interface_node.hpp"
#include "pegasus_utils/frames.hpp"
#include "pegasus_utils/rotations.hpp"

XRCEInterfaceNode::XRCEInterfaceNode(const std::string & node_name, const rclcpp::NodeOptions & options) : rclcpp::Node(node_name, options) {

    // Initialize the parameters
    init_parameters();

    // Attemp to initialize the thrustcurve object such that a controller can specify 
    // to the driver the total thrust to apply to the vehicle in (N) and the conversion is made implicitly
    // to a percentage of the maximum thrust that the vehicle is capable of outputing [0-100%]
    try{
        init_thrust_curve();
    } catch(const std::runtime_error &error) {
        RCLCPP_WARN_STREAM(this->get_logger(), error.what());
        RCLCPP_WARN_STREAM(this->get_logger(), "Could not initilize thrust curve. The xrce driver will only be able to receive the desired thrust in percentage topics");
    }

    // Initialize the publishers, subscribers and services
    initialize_publishers();
    intialize_subscribers();
    initialize_services();

    RCLCPP_INFO_STREAM(this->get_logger(), "XRCE Interface Node initialized successfully");
}

XRCEInterfaceNode::~XRCEInterfaceNode() {

}

void XRCEInterfaceNode::init_parameters() {

    // Vehicle identification (used for frame_ids and namespacing if needed)
    this->declare_parameter<int>("vehicle_id", 1);
    vehicle_id_ = this->get_parameter("vehicle_id").as_int();

    this->declare_parameter<std::string>("vehicle_ns", "drone");
    vehicle_ns_ = this->get_parameter("vehicle_ns").as_string();
}

void XRCEInterfaceNode::initialize_publishers() {

    // Declare the parameters for the topics to publish on
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.actuator_setpoints", "actuator_setpoints");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.attitude_setpoints", "attitude_setpoints");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.thrust_setpoints", "thrust_setpoints");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.torque_setpoints", "torque_setpoints");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.offboard_control_mode", "offboard_control_mode");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.vehicle_command", "vehicle_command");
    this->declare_parameter<std::string>("xrce_interface.px4.publishers.mocap", "mocap_odometry");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.status", "status");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.vehicle_constants", "vehicle_constants");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.sensors.imu", "sensors/imu");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.sensors.gps", "sensors/gps");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.sensors.gps_info", "sensors/gps_info");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.filter.state", "filter/state");
    this->declare_parameter<std::string>("xrce_interface.pegasus.publishers.filter.rpy", "filter/rpy");

    // ----- PX4 Publishers -----
    actuator_motors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(this->get_parameter("xrce_interface.px4.publishers.actuator_setpoints").as_string(), 10);
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(this->get_parameter("xrce_interface.px4.publishers.attitude_setpoints").as_string(), 10);  
    thrust_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(this->get_parameter("xrce_interface.px4.publishers.thrust_setpoints").as_string(), 10);
    torque_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(this->get_parameter("xrce_interface.px4.publishers.torque_setpoints").as_string(), 10);
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(this->get_parameter("xrce_interface.px4.publishers.offboard_control_mode").as_string(), 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(this->get_parameter("xrce_interface.px4.publishers.vehicle_command").as_string(), 10);

    // Mocap pose publisher for the PX4 estimator
    mocap_odometry_px4_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(this->get_parameter("xrce_interface.px4.publishers.mocap").as_string(), 10);

    // ----- Pegasus Publishers -----
    
    // ------------------------------------------------------------------------
    // Initialize the publisher for the status of the vehicle (arm/disarm state, connection, ...)
    // ------------------------------------------------------------------------
    status_pub_ = this->create_publisher<pegasus_msgs::msg::Status>(this->get_parameter("xrce_interface.pegasus.publishers.status").as_string(), rclcpp::SensorDataQoS());
    vehicle_constants_pub_ = this->create_publisher<pegasus_msgs::msg::VehicleConstants>(this->get_parameter("xrce_interface.pegasus.publishers.vehicle_constants").as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for sensors data (IMU and gps)
    // ------------------------------------------------------------------------
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(this->get_parameter("xrce_interface.pegasus.publishers.sensors.imu").as_string(), rclcpp::SensorDataQoS());
    gps_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGps>(this->get_parameter("xrce_interface.pegasus.publishers.sensors.gps").as_string(), rclcpp::SensorDataQoS());
    gps_info_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGpsInfo>(this->get_parameter("xrce_interface.pegasus.publishers.sensors.gps_info").as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for the current state of the vehicle 
    // (position, orientation, body and inertial frame velocity)
    // ------------------------------------------------------------------------
    filter_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("xrce_interface.pegasus.publishers.filter.state").as_string(), rclcpp::SensorDataQoS());
    filter_state_rpy_pub_ = this->create_publisher<pegasus_msgs::msg::RPY>(this->get_parameter("xrce_interface.pegasus.publishers.filter.rpy").as_string(), rclcpp::SensorDataQoS());
}

void XRCEInterfaceNode::intialize_subscribers() {

    // Declare the parameters for the topics to subscribe to
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.status", "vehicle_status");
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.battery", "battery_status");
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.odometry", "vehicle_odometry");
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.imu", "vehicle_imu");
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.attitude", "vehicle_attitude");
    this->declare_parameter<std::string>("xrce_interface.px4.subscribers.gps", "vehicle_gps");

    // Mocap data subscriber
    this->declare_parameter<std::string>("xrce_interface.pegasus.subscribers.external_sensors.mocap_enu", "mocap");

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // ---- PX4 Subscribers ----
    vehicle_status_px4_sub_= this->create_subscription<px4_msgs::msg::VehicleStatus>(this->get_parameter("xrce_interface.px4.subscribers.status").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_status_callback, this, std::placeholders::_1));
    battery_status_px4_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(this->get_parameter("xrce_interface.px4.subscribers.battery").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_battery_callback, this, std::placeholders::_1));
    
    vehicle_odometry_px4_sub_= this->create_subscription<px4_msgs::msg::VehicleOdometry>(this->get_parameter("xrce_interface.px4.subscribers.odometry").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_odometry_callback, this, std::placeholders::_1));
    vehicle_imu_px4_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(this->get_parameter("xrce_interface.px4.subscribers.imu").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_imu_callback, this, std::placeholders::_1));
    vehicle_attitude_px4_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(this->get_parameter("xrce_interface.px4.subscribers.attitude").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_attitude_callback, this, std::placeholders::_1));
    vehicle_gps_px4_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(this->get_parameter("xrce_interface.px4.subscribers.gps").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_gps_callback, this, std::placeholders::_1));

    // Mocap subscriber
    mocap_pose_enu_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("xrce_interface.pegasus.subscribers.external_sensors.mocap_enu").as_string()  + "/" + vehicle_ns_ + std::to_string(vehicle_id_),
        rclcpp::SensorDataQoS(), std::bind(&XRCEInterfaceNode::mocap_pose_callback, this, std::placeholders::_1));
}

void XRCEInterfaceNode::initialize_services() {

    //Declare the parameters for the services to offer
    this->declare_parameter<std::string>("xrce_interface.pegasus.services.arm", "arm");
    this->declare_parameter<std::string>("xrce_interface.pegasus.services.kill_switch", "kill_switch");
    this->declare_parameter<std::string>("xrce_interface.pegasus.services.land", "land");
    this->declare_parameter<std::string>("xrce_interface.pegasus.services.offboard", "offboard");
    this->declare_parameter<std::string>("xrce_interface.pegasus.services.hold", "position_hold");

    // Pegasus services to interface with PX4
    arm_service_ = this->create_service<pegasus_msgs::srv::Arm>(this->get_parameter("xrce_interface.pegasus.services.arm").as_string(), std::bind(&XRCEInterfaceNode::arm_callback, this, std::placeholders::_1, std::placeholders::_2));
    kill_switch_service_ = this->create_service<pegasus_msgs::srv::KillSwitch>(this->get_parameter("xrce_interface.pegasus.services.kill_switch").as_string(), std::bind(&XRCEInterfaceNode::kill_switch_callback, this, std::placeholders::_1, std::placeholders::_2));
    land_service_ = this->create_service<pegasus_msgs::srv::Land>(this->get_parameter("xrce_interface.pegasus.services.land").as_string(), std::bind(&XRCEInterfaceNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));
    offboard_service_ = this->create_service<pegasus_msgs::srv::Offboard>(this->get_parameter("xrce_interface.pegasus.services.offboard").as_string(), std::bind(&XRCEInterfaceNode::offboard_callback, this, std::placeholders::_1, std::placeholders::_2));
    position_hold_service_ = this->create_service<pegasus_msgs::srv::PositionHold>(this->get_parameter("xrce_interface.pegasus.services.hold").as_string(), std::bind(&XRCEInterfaceNode::position_hold_callback, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Method used to initialize the ThrustCurve object used
 * to translate the thrust curve from Newton (N) to percentage (0-100%) and vice-versa
 * based on the configurations loaded from ROS parameter server
 */
void XRCEInterfaceNode::init_thrust_curve() {
    
    // Get the singleton factory of thrust curves
    auto thrust_curve_Factory = Pegasus::ThrustCurveFactory::get_instance();

    RCLCPP_WARN_STREAM(this->get_logger(), "Initializing the thrust curve with the following parameters:");

    // Get from the ROS parameter server the mass of the vehicle
    this->declare_parameter<double>("dynamics.mass", 0.0);
    rclcpp::Parameter mass = this->get_parameter("dynamics.mass");

    
    RCLCPP_WARN_STREAM(this->get_logger(), "Vehicle mass set to: " << mass.as_double() << " Kg");

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

void XRCEInterfaceNode::px4_imu_callback(px4_msgs::msg::SensorCombined::ConstSharedPtr imu_msg) {

    // Set the IMU message with the data received from the PX4 autopilot
    imu_msg_.header.stamp = rclcpp::Clock().now();
    imu_msg_.header.frame_id = "base_link_ned";

    imu_msg_.angular_velocity.x = imu_msg->gyro_rad[0];
    imu_msg_.angular_velocity.y = imu_msg->gyro_rad[1];
    imu_msg_.angular_velocity.z = imu_msg->gyro_rad[2];

    imu_msg_.linear_acceleration.x = imu_msg->accelerometer_m_s2[0];
    imu_msg_.linear_acceleration.y = imu_msg->accelerometer_m_s2[1];
    imu_msg_.linear_acceleration.z = imu_msg->accelerometer_m_s2[2];

    // Publish the IMU message according to the Pegasus API
    imu_pub_->publish(imu_msg_);
}

void XRCEInterfaceNode::px4_attitude_callback(px4_msgs::msg::VehicleAttitude::ConstSharedPtr attitude_msg) {

    // Set the attitude setpoint message with the data received from the PX4 autopilot
    imu_msg_.orientation.w = attitude_msg->q[0];
    imu_msg_.orientation.x = attitude_msg->q[1];
    imu_msg_.orientation.y = attitude_msg->q[2];
    imu_msg_.orientation.z = attitude_msg->q[3];

    // Publish the IMU message according to the Pegasus API
    imu_pub_->publish(imu_msg_);
}

void XRCEInterfaceNode::px4_gps_callback(px4_msgs::msg::SensorGps::ConstSharedPtr gps_msg) {

    gps_msg_.header.stamp = rclcpp::Clock().now();
    gps_msg_.header.frame_id = "base_link_ned";

    // Set the GPS fields
    gps_msg_.latitude_deg = gps_msg->latitude_deg;
    gps_msg_.longitude_deg = gps_msg->longitude_deg;
    gps_msg_.altitude_msl = gps_msg->altitude_msl_m;
    gps_msg_.altitude_ellipsoid = gps_msg->altitude_ellipsoid_m;
    gps_msg_.hdop = gps_msg->hdop;
    gps_msg_.vdop = gps_msg->vdop;
    gps_msg_.velocity = gps_msg->vel_m_s;
    gps_msg_.heading = gps_msg->heading;
    gps_msg_.cog_deg = gps_msg->cog_rad * 180.0 / M_PI; // Convert from radians to degrees
    gps_msg_.horizontal_uncertainty = gps_msg->eph;
    gps_msg_.vertical_uncertainty = gps_msg->epv;
    gps_msg_.velocity_uncertainty = gps_msg->s_variance_m_s;
    gps_msg_.heading_uncertainty = gps_msg->c_variance_rad * 180.0 / M_PI; // Convert from radians to degrees

    // Update the GPS info message with the number of satellites and the fix type
    gps_info_msg_.header.stamp = gps_msg_.header.stamp;
    gps_info_msg_.header.frame_id = "base_link_ned";

    gps_info_msg_.num_satellites = gps_msg->satellites_used;
    gps_info_msg_.fix_type = gps_msg->fix_type;

    // Publish the GPS message
    gps_pub_->publish(gps_msg_);

    // Publish the GPS info message
    gps_info_pub_->publish(gps_info_msg_);
}

void XRCEInterfaceNode::px4_odometry_callback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr odom_msg) {

    // Set the filter state message with the odometry data received from the PX4 autopilot
    filter_state_msg_.header.stamp = rclcpp::Clock().now();
    filter_state_msg_.header.frame_id = "map_ned";
    filter_state_msg_.child_frame_id = "base_link_ned";

    filter_state_msg_.pose.pose.position.x = odom_msg->position[0];
    filter_state_msg_.pose.pose.position.y = odom_msg->position[1];
    filter_state_msg_.pose.pose.position.z = odom_msg->position[2];

    filter_state_msg_.pose.pose.orientation.w = odom_msg->q[0];
    filter_state_msg_.pose.pose.orientation.x = odom_msg->q[1];
    filter_state_msg_.pose.pose.orientation.y = odom_msg->q[2];
    filter_state_msg_.pose.pose.orientation.z = odom_msg->q[3];

    filter_state_msg_.twist.twist.linear.x = odom_msg->velocity[0];
    filter_state_msg_.twist.twist.linear.y = odom_msg->velocity[1];
    filter_state_msg_.twist.twist.linear.z = odom_msg->velocity[2];

    filter_state_msg_.twist.twist.angular.x = odom_msg->angular_velocity[0];
    filter_state_msg_.twist.twist.angular.y = odom_msg->angular_velocity[1];
    filter_state_msg_.twist.twist.angular.z = odom_msg->angular_velocity[2];

    // Create the Eigen quaternion object and convert the angle to roll, pitch and yaw (using Z-Y-X convention)
    Eigen::Vector3d euler_angles = Pegasus::Rotations::quaternion_to_euler(Eigen::Quaterniond(odom_msg->q[0], odom_msg->q[1], odom_msg->q[2], odom_msg->q[3]));

    // Update the filter state RPY message with the roll, pitch and yaw angles extracted from the odometry message
    filter_state_rpy_msg_.header.stamp = filter_state_msg_.header.stamp;
    filter_state_rpy_msg_.roll = Pegasus::Rotations::rad_to_deg(euler_angles(0));
    filter_state_rpy_msg_.pitch = Pegasus::Rotations::rad_to_deg(euler_angles(1));
    filter_state_rpy_msg_.yaw = Pegasus::Rotations::rad_to_deg(euler_angles(2));

    // Publish the filter state message according to the Pegasus API
    filter_state_pub_->publish(filter_state_msg_);
    filter_state_rpy_pub_->publish(filter_state_rpy_msg_);
}

void XRCEInterfaceNode::px4_status_callback(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {

    const std::map<int, int> PX4_modes {
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL, pegasus_msgs::msg::Status::MANUAL},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL, pegasus_msgs::msg::Status::ATL_CTL},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL, pegasus_msgs::msg::Status::POS_CTL},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION, pegasus_msgs::msg::Status::MISSION},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER, pegasus_msgs::msg::Status::HOLD},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL, pegasus_msgs::msg::Status::RETURN_TO_LAUNCH},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO, pegasus_msgs::msg::Status::ACRO},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD, pegasus_msgs::msg::Status::OFFBOARD},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF, pegasus_msgs::msg::Status::TAKEOFF},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND, pegasus_msgs::msg::Status::LAND},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET, pegasus_msgs::msg::Status::FOLLOW_ME},
        {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB, pegasus_msgs::msg::Status::STABILIZED},
    };

    // Set the status message with the status data received from the PX4 autopilot
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.header.frame_id = "base_link_ned";

    // Base flight information
    status_msg_.system_id = msg->system_id;
    status_msg_.armed = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) ? true : false;
    status_msg_.landed_state = (msg->takeoff_time > 0) ? status_msg_.IN_AIR : status_msg_.ON_GROUND;
    status_msg_.flight_mode = (PX4_modes.find(msg->nav_state) != PX4_modes.end()) ? PX4_modes.at(msg->nav_state) : status_msg_.UNKOWN;

    // Update the landed state with taking off or landing if that is the case
    if(msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
        status_msg_.landed_state = status_msg_.TAKING_OFF;
    } else if(msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        status_msg_.landed_state = status_msg_.LANDING;    
    }

    // Get the health of the vehicle
    status_msg_.health.is_armable = msg->pre_flight_checks_pass;
    status_msg_.health.accelerometer_calibrated = true;
    status_msg_.health.magnetometer_calibrated = true;

    // Publish the status message according to the Pegasus API
    status_pub_->publish(status_msg_);

    // Also publish the message with the vehicle constants
    // TODO: improve this later on
    vehicle_constants_pub_->publish(vehicle_constants_msg_);
}

void XRCEInterfaceNode::px4_battery_callback(px4_msgs::msg::BatteryStatus::ConstSharedPtr battery_msg) {

    // Update the battery status
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.header.frame_id = "base_link_ned";
    status_msg_.battery.id = battery_msg->id;
    status_msg_.battery.temperature = battery_msg->temperature;
    status_msg_.battery.voltage = battery_msg->ocv_estimate_filtered;
    status_msg_.battery.percentage = battery_msg->remaining * 100.0; // Convert from [0-1] to percentage
    status_msg_.battery.current = battery_msg->current_a;
    status_msg_.battery.amps_hour_consumed = battery_msg->discharged_mah;

    // Publish the status message according to the Pegasus API
    status_pub_->publish(status_msg_);
}

/**
 * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
 * @param msg A message with the desired position for the vehicle in NED
 */
void XRCEInterfaceNode::position_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg) {
    
    // Set the offboard mode for position control
    offboard_control_mode_msg_.position = true;
    offboard_control_mode_msg_.velocity = false;
    offboard_control_mode_msg_.acceleration = false;
    offboard_control_mode_msg_.attitude = false;
    offboard_control_mode_msg_.body_rate = false;
    offboard_control_mode_msg_.thrust_and_torque = false;
    offboard_control_mode_msg_.direct_actuator = false;
	offboard_control_mode_msg_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Set the position trajectory message
    trajectory_setpoint_msg_.position = {static_cast<float>(msg->position[0]), static_cast<float>(msg->position[1]), static_cast<float>(msg->position[2])};
	trajectory_setpoint_msg_.yaw = -3.14; // [-PI:PI]
	trajectory_setpoint_msg_.timestamp = offboard_control_mode_msg_.timestamp;

    // Publish the offboard control mode message with the target position message
	offboard_control_mode_pub_->publish(offboard_control_mode_msg_);
    //trajectory_setpoint_pub_->publish(trajectory_setpoint_msg_);
}

/**
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude and thrust to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {

    // Set the offboard mode for attitude control
    offboard_control_mode_msg_.position = false;
    offboard_control_mode_msg_.velocity = false;
    offboard_control_mode_msg_.acceleration = false;
    offboard_control_mode_msg_.attitude = true;
    offboard_control_mode_msg_.body_rate = false;
    offboard_control_mode_msg_.thrust_and_torque = false;
    offboard_control_mode_msg_.direct_actuator = false;
	offboard_control_mode_msg_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Publish the offboard control mode message with the target position message
	offboard_control_mode_pub_->publish(offboard_control_mode_msg_);
}

/**
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_rate_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    
    // Set the offboard mode for attitude control
    offboard_control_mode_msg_.position = false;
    offboard_control_mode_msg_.velocity = false;
    offboard_control_mode_msg_.acceleration = false;
    offboard_control_mode_msg_.attitude = false;
    offboard_control_mode_msg_.body_rate = true;
    offboard_control_mode_msg_.thrust_and_torque = false;
    offboard_control_mode_msg_.direct_actuator = false;
	offboard_control_mode_msg_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Publish the offboard control mode message with the target position message
	offboard_control_mode_pub_->publish(offboard_control_mode_msg_);
}

/**
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude and force to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {

    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    // Send the attitude-rate and thrust reference thorugh XRCE for the onboard microcontroller
    
    // Set the offboard mode for attitude control
    offboard_control_mode_msg_.position = false;
    offboard_control_mode_msg_.velocity = false;
    offboard_control_mode_msg_.acceleration = false;
    offboard_control_mode_msg_.attitude = true;
    offboard_control_mode_msg_.body_rate = false;
    offboard_control_mode_msg_.thrust_and_torque = false;
    offboard_control_mode_msg_.direct_actuator = false;
	offboard_control_mode_msg_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Publish the offboard control mode message with the target position message
	offboard_control_mode_pub_->publish(offboard_control_mode_msg_);
}

/**
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude-rate and force to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_rate_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    
    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    // Send the attitude-rate and thrust reference thorugh XRCE for the onboard microcontroller

    // Set the offboard mode for attitude control
    offboard_control_mode_msg_.position = false;
    offboard_control_mode_msg_.velocity = false;
    offboard_control_mode_msg_.acceleration = false;
    offboard_control_mode_msg_.attitude = false;
    offboard_control_mode_msg_.body_rate = true;
    offboard_control_mode_msg_.thrust_and_torque = false;
    offboard_control_mode_msg_.direct_actuator = false;
	offboard_control_mode_msg_.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Publish the offboard control mode message with the target position message
	offboard_control_mode_pub_->publish(offboard_control_mode_msg_);
}

/**
 * @brief Arming/disarming service callback. When a service request is reached from the arm_service_, 
 * this callback is called and will send a command for the vehicle to arm/disarm
 * @param request The request for arming (bool = true) or disarming (bool = false)
 * @param response The response in this service uint8
 */
void XRCEInterfaceNode::arm_callback(const pegasus_msgs::srv::Arm::Request::SharedPtr request, const pegasus_msgs::srv::Arm::Response::SharedPtr response) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    response->success = true;
}

/**
 * @brief Kill switch service callback. When a service request is reached from the kill_switch_service_,
 * this callback is called and will send a command for the vehicle to kill the motors instantly.
 * @param request The request for killing the motors (bool = true)
 * @param response The response in this service uint8
*/
void XRCEInterfaceNode::kill_switch_callback(const pegasus_msgs::srv::KillSwitch::Request::SharedPtr request, const pegasus_msgs::srv::KillSwitch::Response::SharedPtr response) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    response->success = true;
}

/**
 * @brief Autoland service callback. When a service request is reached from the land_service_,
 * this callback is called and will send a command for the vehicle to autoland using the onboard controller
 * @param request An empty request for landing the vehicle (can be ignored)
 * @param response The response in this service uint8
 */
void XRCEInterfaceNode::land_callback(const pegasus_msgs::srv::Land::Request::SharedPtr request, const pegasus_msgs::srv::Land::Response::SharedPtr response) {
    // Check this page for mode values: https://github.com//MAVSDK/blob/ce6b7186d837b1ab5e9b23bb9be72aec28899630/src/mavsdk/core/px4_custom_mode.h
    // https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/11
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 3, 6); // Set Auto (mode) + Land (submode)
}

/**
 * @brief Offboard service callback. When a service request is reached from the offboard_service_,
 * this callback is called and will send a command for the vehicle to enter offboard mode
 * @param request An empty request for entering offboard mode (can be ignored)
 * @param response The response in this service uint8
 */
void XRCEInterfaceNode::offboard_callback(const pegasus_msgs::srv::Offboard::Request::SharedPtr, const pegasus_msgs::srv::Offboard::Response::SharedPtr response) { 
    // Check this page for mode values: https://github.com//MAVSDK/blob/ce6b7186d837b1ab5e9b23bb9be72aec28899630/src/mavsdk/core/px4_custom_mode.h
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Position hold service callback. When a service request is reached from the position_hold_service_,
 * this callback is called and will send a command for the vehicle to enter position hold mode
 * @param request An empty request for entering position hold mode (can be ignored)
 * @param response The response in this service uint8
 */
void XRCEInterfaceNode::position_hold_callback(const pegasus_msgs::srv::PositionHold::Request::SharedPtr, const pegasus_msgs::srv::PositionHold::Response::SharedPtr response) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 3, 3); // Set Auto (mode) + Hold/Loiter (submode)
}

/**
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via  to the vehicle autopilot filter to merge
 * @param msg A message with the pose of the vehicle expressed in ENU
 */
void XRCEInterfaceNode::mocap_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    
    // Convert the position expressed in ENU {East-North-Up} to NED {North-East-Down}
    Eigen::Vector3d position_ned = Pegasus::Frames::transform_vect_inertial_enu_ned(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    // Convert the orientation of a F.L.U vehicle with respect to ENU {East-North-Up} to
    // F.R.D with respect to NED {North-East-Down}
    Eigen::Quaterniond orientation_flu_enu;
    orientation_flu_enu.x() = msg->pose.orientation.x;
    orientation_flu_enu.y() = msg->pose.orientation.y;
    orientation_flu_enu.z() = msg->pose.orientation.z;
    orientation_flu_enu.w() = msg->pose.orientation.w;
    Eigen::Quaterniond orientation_frd_ned = Pegasus::Frames::rot_body_to_inertial(Eigen::Quaternion<double>(orientation_flu_enu));

    // Send the mocap measured vehicle pose thorugh  for the onboard microcontroller to fuse in its internal EKF
    mocap_odometry_msg_.timestamp = (int) this->get_clock()->now().nanoseconds() / 1000;
    mocap_odometry_msg_.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Set the position in NED
    mocap_odometry_msg_.position[0] = static_cast<float>(position_ned(0));
    mocap_odometry_msg_.position[1] = static_cast<float>(position_ned(1));
    mocap_odometry_msg_.position[2] = static_cast<float>(position_ned(2));
    
    // Set the attitude of F.R.D frame relative to NED frame, expressed as a quaternion
    mocap_odometry_msg_.q[0] = static_cast<float>(orientation_frd_ned.w());
    mocap_odometry_msg_.q[1] = static_cast<float>(orientation_frd_ned.x());
    mocap_odometry_msg_.q[2] = static_cast<float>(orientation_frd_ned.y());
    mocap_odometry_msg_.q[3] = static_cast<float>(orientation_frd_ned.z());

    // Set an unknwon velocity since the motion capture system only provides the pose of the vehicle and not its velocity
    mocap_odometry_msg_.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    mocap_odometry_msg_.velocity[0] = NAN;
    mocap_odometry_msg_.velocity[1] = NAN;
    mocap_odometry_msg_.velocity[2] = NAN;

    // Set an unknown angular velocity since the motion capture system only provides the pose of the vehicle and not its angular velocity
    mocap_odometry_msg_.angular_velocity[0] = NAN;
    mocap_odometry_msg_.angular_velocity[1] = NAN;
    mocap_odometry_msg_.angular_velocity[2] = NAN;

    mocap_odometry_msg_.position_variance[0] = NAN;
    mocap_odometry_msg_.position_variance[1] = NAN;
    mocap_odometry_msg_.position_variance[2] = NAN;

    mocap_odometry_msg_.orientation_variance[0] = NAN;
    mocap_odometry_msg_.orientation_variance[1] = NAN;
    mocap_odometry_msg_.orientation_variance[2] = NAN;

    mocap_odometry_msg_.velocity_variance[0] = NAN;
    mocap_odometry_msg_.velocity_variance[1] = NAN;
    mocap_odometry_msg_.velocity_variance[2] = NAN;

    // Publish the mocap odometry message according to the PX4 API
    mocap_odometry_px4_pub_->publish(mocap_odometry_msg_);
}

void XRCEInterfaceNode::publish_vehicle_command(uint16_t command, float param1, float param2) {

	px4_msgs::msg::VehicleCommand msg;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}