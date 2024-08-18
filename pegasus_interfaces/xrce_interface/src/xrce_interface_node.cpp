#include "xrce_interface_node.hpp"

XRCEInterfaceNode::XRCEInterfaceNode() : rclcpp::Node("xrce_interface_node") {

}

XRCEInterfaceNode::~XRCEInterfaceNode() {

}

void XRCEInterfaceNode::init_parameters() {

}

void XRCEInterfaceNode::initialize_publishers() {

    // ----- PX4 Publishers -----
    actuator_motors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(this->get_parameter("xrce_interface.px4.publishers.actuator_setpoints").as_string(), 10);
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(this->get_parameter("xrce_interface.px4.publishers.attitude_setpoints").as_string(), 10);  
    thrust_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(this->get_parameter("xrce_interface.px4.publishers.thrust_setpoints").as_string(), 10);
    torque_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(this->get_parameter("xrce_interface.px4.publishers.torque_setpoints").as_string(), 10);
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(this->get_parameter("xrce_interface.px4.publishers.offboard_control_mode").as_string(), 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(this->get_parameter("xrce_interface.px4.publishers.vehicle_command").as_string(), 10);

    // ----- Pegasus Publishers -----
    
    // ------------------------------------------------------------------------
    // Initialize the publisher for the status of the vehicle (arm/disarm state, connection, ...)
    // ------------------------------------------------------------------------
    status_pub_ = this->create_publisher<pegasus_msgs::msg::Status>(this->get_parameter("xrce_interface.publishers.status").as_string(), rclcpp::SensorDataQoS());
    vehicle_constants_pub_ = this->create_publisher<pegasus_msgs::msg::VehicleConstants>(this->get_parameter("xrce_interface.publishers.vehicle_constants").as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for sensors data (IMU, barometer and gps)
    // ------------------------------------------------------------------------
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(this->get_parameter("xrce_interface.publishers.sensors.imu").as_string(), rclcpp::SensorDataQoS());
    baro_pub_ = this->create_publisher<pegasus_msgs::msg::SensorBarometer>(this->get_parameter("xrce_interface.publishers.sensors.barometer").as_string(), rclcpp::SensorDataQoS());
    gps_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGps>(this->get_parameter("xrce_interface.publishers.sensors.gps").as_string(), rclcpp::SensorDataQoS());
    gps_info_pub_ = this->create_publisher<pegasus_msgs::msg::SensorGpsInfo>(this->get_parameter("xrce_interface.publishers.sensors.gps_info").as_string(), rclcpp::SensorDataQoS());

    // ------------------------------------------------------------------------
    // Initialize the publisher for the current state of the vehicle 
    // (position, orientation, body and inertial frame velocity)
    // ------------------------------------------------------------------------
    filter_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("xrce_interface.publishers.filter.state").as_string(), rclcpp::SensorDataQoS());
    filter_state_rpy_pub_ = this->create_publisher<pegasus_msgs::msg::RPY>(this->get_parameter("xrce_interface.publishers.filter.rpy").as_string(), rclcpp::SensorDataQoS());
}

void XRCEInterfaceNode::intialize_subscribers() {

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // ---- PX4 Subscribers ----
    vehicle_odometry_sub_= this->create_subscription<px4_msgs::msg::VehicleOdometry>(this->get_parameter("xrce_interface.px4.subscribers.odometry").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_odometry_callback, this, _1));
    vehicle_status_sub_= this->create_subscription<px4_msgs::msg::VehicleStatus>(this->get_parameter("xrce_interface.px4.subscribers.status").as_string(), qos, std::bind(&XRCEInterfaceNode::px4_status_callback, this, _1));
}

void XRCEInterfaceNode::initialize_services() {

}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize the ThrustCurve object used
 * to translate the thrust curve from Newton (N) to percentage (0-100%) and vice-versa
 * based on the configurations loaded from ROS parameter server
 */
void XRCEInterfaceNode::init_thrust_curve() {
    
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

void XRCEInterfaceNode::px4_odometry_callback(px4_msgs::msg::VehicleOdometry::ConstSharedPtr odom_msg) {

    // Set the filter state message with the odometry data received from the PX4 autopilot
    filter_state_msg_.header.stamp = rclcpp::Clock().now();
    filter_state_msg_.header.frame_id = "map";
    filter_state_msg_.child_frame_id = "base_link";

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

    // Publish the filter state message according to the Pegasus API
    filter_state_pub_->publish(filter_state_msg_);
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
    status_msg_.header.frame_id = "map";

    status_msg_.system_id = msg->system_id;
    status_msg_.armed = (msg->arming_state == 2) ? true : false;
    status_msg_.landed_state = (msg->takeoff_time == 0) ? status_msg_.ON_GROUND : status_msg_.IN_AIR;
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
}

/**
 * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
 * @param msg A message with the desired position for the vehicle in NED
 */
void XRCEInterfaceNode::position_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg) {
    // Send the position reference through the XRCE interface
    // TODO
}

/**
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude and thrust to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    // Send the attitude and thrust reference thorugh XRCE for the onboard microcontroller
    // TODO
}

/**
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
 */
void XRCEInterfaceNode::attitude_rate_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg) {
    // Send the attitude-rate and thrust reference thorugh XRCE for the onboard microcontroller
    // TODO
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
    // TODO
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
}

/**
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via mavlink to the vehicle autopilot filter to merge
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

    Eigen::Vector3d orientation_frd_ned = Pegasus::Rotations::quaternion_to_euler(
        Pegasus::Frames::rot_body_to_inertial(
            Eigen::Quaternion<double>(orientation_flu_enu)));

    // Send the mocap measured vehicle pose thorugh mavlink for the onboard microcontroller
    // to fuse in its internal EKF
    mavlink_node_->update_mocap_telemetry(position_ned, orientation_frd_ned);
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