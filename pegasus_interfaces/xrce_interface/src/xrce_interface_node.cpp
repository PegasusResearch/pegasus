#include "xrce_interface_node.hpp"

XRCEInterfaceNode::XRCEInterfaceNode() : rclcpp::Node("xrce_interface_node") {

}

XRCEInterfaceNode::~XRCEInterfaceNode() {

}

void XRCEInterfaceNode::init_parameters() {

}

void XRCEInterfaceNode::initialize_publishers() {
    
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

void XRCEInterfaceNode::intialize_subscribers() {

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

/**
 * @defgroup subscriberCallbacks
 * This group defines all the ROS subscriber callbacks
 */

/**
 * @ingroup subscriberCallbacks
 * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
 * @param msg A message with the desired position for the vehicle in NED
 */
void XRCEInterfaceNode::position_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg) {
    // Send the position reference through the XRCE interface
    // TODO
}

/**
 * @ingroup subscriberCallbacks
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
 * @ingroup subscriberCallbacks
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
 * @ingroup subscriberCallbacks
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
 * @ingroup subscriberCallbacks
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
 * @ingroup subscriberCallbacks
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