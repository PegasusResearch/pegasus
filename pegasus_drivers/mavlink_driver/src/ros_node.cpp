#include <Eigen/Dense>
#include "ros_node.hpp"
#include "mavlink_node.hpp"
#include "pegasus_utils/frames.hpp"

/**
 * @brief Constructor for the ROS2 node that receives control commands 
 * from ROS and send sensor data to ROS2
 * @param nh The ROS2 nodehandler necessary to read parameters from the node parameter server
 */
ROSNode::ROSNode(rclcpp::Node::SharedPtr nh) : nh_(nh) {

    // Initialize the mass of the vehicle, by reading this parameter from the parameter server
    init_mass();

    // Attemp to initialize the thrustcurve object such that a controller can specify 
    // to the driver the total thrust to apply to the vehicle in (N) and the conversion is made implicitly
    // to a percentage of the maximum thrust that the vehicle is capable of outputing [0-100%]
    try{
        init_thrust_curve();
    } catch(const std::runtime_error &error) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), error.what());
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Could not initilize thrust curve. The mavlink driver will only be able to receive the desired thrust in percentage topics");
    }
}

/**
 * @brief Destroy the ROSNode object
 */
ROSNode::~ROSNode() {}

/**
 * @brief Method used to initialize the mavlink_node object pointer
 * @param mavlink_node A pointer to a mavlink_node object
 */
void ROSNode::init_mavlink_node(const std::shared_ptr<MavlinkNode> mavlink_node) {
    mavlink_node_ = mavlink_node;
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 parameters that we are supposed to read
 * from the parameter server
 */
void ROSNode::init_dynamic_parameters() {

}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 publishers
 */
void ROSNode::init_publishers() {
    
    // ------------------------------------------------------------------------
    // Initialize the publisher for the status of the vehicle (arm/disarm state, connection, ...)
    // ------------------------------------------------------------------------
    nh_->declare_parameter("publishers.status", "status");
    rclcpp::Parameter status_topic = nh_->get_parameter("publishers.status");
    status_pub_ = nh_->create_publisher<pegasus_msgs::msg::Status>(status_topic.as_string(), 1);

    // ------------------------------------------------------------------------
    // Initialize the publisher for the current state of the vehicle 
    // (position, orientation, body and inertial frame velocity)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("publishers.nav.state", "nav/state");
    rclcpp::Parameter state_topic = nh_->get_parameter("publishers.nav.state");
    state_pub_ = nh_->create_publisher<pegasus_msgs::msg::State>(state_topic.as_string(), 1);

    // ------------------------------------------------------------------------
    // Initialize the publisher for the IMU driver data
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("publishers.sensors.imu.vel_and_accel", "sensors/imu/vel_and_accel");
    rclcpp::Parameter imu_vel_accel_topic = nh_->get_parameter("publishers.sensors.imu.vel_and_accel");
    imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(imu_vel_accel_topic.as_string(), 1);
    
    nh_->declare_parameter<std::string>("publishers.sensors.imu.magnetic_field", "sensors/imu/magnetic_field");
    rclcpp::Parameter imu_magnetic_field_topic = nh_->get_parameter("publishers.sensors.imu.magnetic_field");
    mag_pub_ = nh_->create_publisher<sensor_msgs::msg::MagneticField>(imu_magnetic_field_topic.as_string(), 1);
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 subscribers
 */
void ROSNode::init_subscribers() {
    
    // ------------------------------------------------------------------------
    // Subscribe to velocity control (in body frame NED) and desired yaw-angle
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("subscribers.control.body_velocity", "control/body_velocity");
    rclcpp::Parameter body_velocity_topic = nh_->get_parameter("subscribers.control.body_velocity");
    body_velocity_sub_ = nh_->create_subscription<pegasus_msgs::msg::BodyVelocityControl>(
        body_velocity_topic.as_string(), 1, std::bind(&ROSNode::body_velocity_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the position control (north-east-down in meters NED) and desired yaw (in deg)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("subscribers.control.position", "control/position"); 
    rclcpp::Parameter position_control_topic = nh_->get_parameter("subscribers.control.position");
    position_control_sub_ = nh_->create_subscription<pegasus_msgs::msg::PositionControl>(
        position_control_topic.as_string(), 1, std::bind(&ROSNode::position_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the attitude (roll, pitch, yaw NED frame) and desired total thrust (0-100%)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("subscribers.control.attitude_thrust", "control/attitude_thrust");
    rclcpp::Parameter attitude_thrust_topic = nh_->get_parameter("subscribers.control.attitude_thrust");
    attitude_thrust_sub_ = nh_->create_subscription<pegasus_msgs::msg::AttitudeThrustControl>(
        attitude_thrust_topic.as_string(), 1, std::bind(&ROSNode::attitude_thrust_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the attitude rate (roll-rate, pitch-rate, yaw-rate in NED frame) and desired total thrust (0-100%)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("subscribers.control.attitude_rate_thrust", "control/attitude_rate_thrust");
    rclcpp::Parameter attitude_rate_thrust_topic = nh_->get_parameter("subscribers.control.attitude_rate_thrust");
    attitude_rate_thrust_sub_ = nh_->create_subscription<pegasus_msgs::msg::AttitudeRateThrustControl>(
        attitude_rate_thrust_topic.as_string(), 1, std::bind(&ROSNode::attitude_rate_thrust_callback, this, std::placeholders::_1));
    
    // If there is a thrust curve object already available, make 2 subcribers for controlling the 
    // attitude or attitude_rate along with the total desired force along the Z-axis expressed in Newton (N)
    // such that someone implementing a controller in another ROS node does not have to worry about the 
    // conversition from Forces to other unit such as percentage
    if(thrust_curve_) {
        nh_->declare_parameter<std::string>("subscribers.control.attitude_force", "control/attitude_force");
        rclcpp::Parameter attitude_force_topic = nh_->get_parameter("subscribers.control.attitude_force");
        attitude_force_sub_ = nh_->create_subscription<pegasus_msgs::msg::AttitudeThrustControl>(
            attitude_force_topic.as_string(), 1, std::bind(&ROSNode::attitude_force_callback, this, std::placeholders::_1));
        
        nh_->declare_parameter<std::string>("subscribers.control.attitude_rate_force", "control/attitude_rate_force");
        rclcpp::Parameter attitude_rate_force_topic = nh_->get_parameter("subscribers.control.attitude_rate_force");
        attitude_rate_force_sub_ = nh_->create_subscription<pegasus_msgs::msg::AttitudeRateThrustControl>(
            attitude_rate_force_topic.as_string(), 1, std::bind(&ROSNode::attitude_rate_force_callback, this, std::placeholders::_1));
    }

    // ------------------------------------------------------------------------
    // Subscribe to data comming from the motion capture system (MOCAP)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("subscribers.external_sensors.mocap_enu", "mocap/pose_enu");
    rclcpp::Parameter mocap_topic = nh_->get_parameter("subscribers.external_sensors.mocap_enu");
    mocap_pose_enu_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_topic.as_string(), 1, std::bind(&ROSNode::mocap_pose_callback, this, std::placeholders::_1));

}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 services
 */
void ROSNode::init_services() {
    
    // ------------------------------------------------------------------------
    // Initiate the service to ARM/Disarm the vehicle
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("services.arm", "arm");
    rclcpp::Parameter arm_topic = nh_->get_parameter("services.arm");
    arm_service_ = nh_->create_service<pegasus_msgs::srv::Arm>(
        arm_topic.as_string(), std::bind(&ROSNode::arm_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to Land the vehicle
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("services.land", "land");
    rclcpp::Parameter land_topic = nh_->get_parameter("services.land");
    land_service_ = nh_->create_service<pegasus_msgs::srv::Land>(
        land_topic.as_string(), std::bind(&ROSNode::land_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to reboot the vehicle's onboard microcontroller (NOT PC!)
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("services.reboot", "reboot");
    rclcpp::Parameter reboot_topic = nh_->get_parameter("services.reboot");
    reboot_service_ = nh_->create_service<pegasus_msgs::srv::Reboot>(
        reboot_topic.as_string(), std::bind(&ROSNode::reboot_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to request the current mass and thrust curve of the vehicle
    // ------------------------------------------------------------------------
    nh_->declare_parameter<std::string>("services.thrust_curve", "thrust_curve");
    rclcpp::Parameter thrust_curve_topic = nh_->get_parameter("services.thrust_curve");
    thrust_curve_service_ = nh_->create_service<pegasus_msgs::srv::ThrustCurve>(
        thrust_curve_topic.as_string(), std::bind(&ROSNode::thrust_curve_callback, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 actions
 */
void ROSNode::init_actions() {
    // Future proofing
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

    // Get from the ROS parameter server the type of thrust curve to apply to the vehicle
    nh_->declare_parameter<std::string>("dynamics.thrust_curve.identifier", "None"); 
    rclcpp::Parameter thrust_curve_id = nh_->get_parameter("dynamics.thrust_curve.identifier");

    // Get from the ROS parameter server the parameters that define the chosen thrust curve
    nh_->declare_parameter<std::vector<std::string>>("dynamics.thrust_curve.parameter_names", std::vector<std::string>()); 
    rclcpp::Parameter thrust_curve_parameter_names = nh_->get_parameter("dynamics.thrust_curve.parameter_names");

    nh_->declare_parameter<std::vector<double>>("dynamics.thrust_curve.parameters", std::vector<double>());
    rclcpp::Parameter thrust_curve_parameters = nh_->get_parameter("dynamics.thrust_curve.parameters");

    // Check if the thrust curve parameter_names has the same size as the number of parameters
    // If not, throw a runtime error
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
 * @brief Method used to initialize the mass of the vehicle (loaded from ROS parameter server)
 */
void ROSNode::init_mass() {
    // Get mass of the vehicle from the ROS parameter server
    nh_->declare_parameter<double>("dynamics.mass", 0.0); 
    rclcpp::Parameter mass = nh_->get_parameter("dynamics.mass");
    mass_ = mass.as_double();

    // Warn the user if the mass parameter is not well setup
    if(mass_ <= 0.0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Mass parameter seems invalid: " + std::to_string(mass_) + " Kg.");
    }
}

/**
 * @defgroup subscriberCallbacks
 * This group defines all the ROS subscriber callbacks
 */

/**
 * @ingroup subscriberCallbacks
 * @brief Body Velocity subscriber callback. For multirotors, the x,y,z axis on the body frame are
 * always assumed by the autopilot to not be affected by roll and pitch, wich means that
 * they lie in a constant plane (otherwise, controlling in these variables would lead the drone to crash)
 * @param msg A message with the desired body velocity to apply to the vehicle
 */
void ROSNode::body_velocity_callback(const pegasus_msgs::msg::BodyVelocityControl::SharedPtr msg) {
    // Send the body velocity reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_body_velocity(
        msg->velocity_body[0], 
        msg->velocity_body[1],
        msg->velocity_body[2],
        msg->yaw_rate_deg);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
 * @param msg A message with the desired position for the vehicle in NED
 */
void ROSNode::position_callback(const pegasus_msgs::msg::PositionControl::SharedPtr msg) {
    // Send the position reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_position(
        msg->position[0],
        msg->position[1],
        msg->position[2],
        msg->yaw);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude and thrust to apply to the vehicle
 */
void ROSNode::attitude_thrust_callback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg) {
    // Send the attitude and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude(
        msg->attitude[0],
        msg->attitude[1],
        msg->attitude[2],
        msg->thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
 * between 0-100 %
 * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
 */
void ROSNode::attitude_rate_thrust_callback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg) {
    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude_rate(
        msg->attitude_rate[0],
        msg->attitude_rate[1],
        msg->attitude_rate[2],
        msg->thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude and force to apply to the vehicle
 */
void ROSNode::attitude_force_callback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg) {

    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    RCLCPP_INFO_STREAM(nh_->get_logger(), thrust);

    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude(
        msg->attitude[0],
        msg->attitude[1],
        msg->attitude[2],
        thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
 * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
 * the body Z-axis should be given in Newton (N)
 * @param msg A message with the desired attitude-rate and force to apply to the vehicle
 */
void ROSNode::attitude_rate_force_callback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg) {
    
    // Convert the force received in the message in Newton (N) to a percentage from [0-100]%
    double thrust = thrust_curve_->force_to_percentage(msg->thrust);

    // Send the attitude-rate and thrust reference thorugh mavlink for the onboard microcontroller
    mavlink_node_->set_attitude_rate(
        msg->attitude_rate[0],
        msg->attitude_rate[1],
        msg->attitude_rate[2],
        thrust);
}

/**
 * @ingroup subscriberCallbacks
 * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
 * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
 * sends it via mavlink to the vehicle autopilot filter to merge
 * @param msg A message with the pose of the vehicle expressed in ENU
 */
void ROSNode::mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    
    // Convert the position expressed in ENU {East-North-Up} to NED {North-East-Down}
    Eigen::Vector3d position_ned = Pegasus::Frames::transform_vect_inertial_enu_ned(
        Eigen::Vector3d(
            msg->pose.position.x, 
            msg->pose.position.y,
            msg->pose.position.z));

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
 * @brief Method that is called to update the system_id field in the status_msg. This method
 * DOES NOT publish the most up to date message to the status_pub
 * @param id An unsigned int (a byte) that encodes the mavlink id of the connected vehicle
 */
void ROSNode::update_system_id(const uint8_t &id) {
    status_msg_.system_id = id;
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the armed field in the status_msg. This method
 * publishes the most up to date message to the status_pub
 * @param is_armed A boolean whether the vehicle is armed(1)/disarmed(0)
 */
void ROSNode::update_armed_state(const bool &is_armed) {
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.armed = is_armed;
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the flight_mode field in the status_msg. This method
 * publishes the most up to date message to the status_pub
 * @param flight_mode An byte that containts the current flightmode active in the onboard microcontroller
 */
void ROSNode::update_flightmode_state(const unsigned char &flight_mode) {
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.flight_mode = flight_mode;
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the remaining battery percentage field in the status_msg. This method
 * publishes the most up to date message to status_pub
 * @param percentage A float with the current battery percentage
 */
void ROSNode::update_battery_state(const float &percentage) {
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.battery = percentage;
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the signal strength of a connected vehicle RC controller field in the status_msg. This method
 * publishes the most up to date message to status_pub
 * @param rc_signal_strength_percentage A float with the current rc signal strength percentage. Negative value means no RC connected
 */
void ROSNode::update_rc_state(const float &rc_signal_strength_percentage) {
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.rc_signal = rc_signal_strength_percentage;
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
void ROSNode::update_heath_state(const mavsdk::Telemetry::Health &health) {
    status_msg_.header.stamp = rclcpp::Clock().now();
    status_msg_.health.is_armable = health.is_armable;
    status_msg_.health.accelerometer_calibrated = health.is_accelerometer_calibration_ok;
    status_msg_.health.magnetometer_calibrated = health.is_magnetometer_calibration_ok;

    // TODO - check if these 3 fields in the message is really necessary or if we can omit them for the sake of simplicity
    status_msg_.health.local_position_ok = health.is_local_position_ok;
    status_msg_.health.global_position_ok = health.is_global_position_ok;
    status_msg_.health.home_position_ok = health.is_home_position_ok;
    status_pub_->publish(status_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the pose and inertial_vel fields in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param pos_vel_ned A mavsdk structure which contains the position and linear velocity of the vehicle expressed in the inertial frame
 * in NED
 */
void ROSNode::update_position_velocity_ned(const mavsdk::Telemetry::PositionVelocityNed &pos_vel_ned) {
    // Update the absolule position of the vehicle in the "pose" message
    state_msg_.pose.header.stamp = rclcpp::Clock().now();
    state_msg_.pose.pose.position.x = pos_vel_ned.position.north_m;
    state_msg_.pose.pose.position.y = pos_vel_ned.position.east_m;
    state_msg_.pose.pose.position.z = pos_vel_ned.position.down_m;

    // Update the inertial velocity of the vehicle x_dot, y_dot, z_dot
    state_msg_.inertial_vel.header.stamp = rclcpp::Clock().now();
    state_msg_.inertial_vel.vector.x = pos_vel_ned.velocity.north_m_s;
    state_msg_.inertial_vel.vector.y = pos_vel_ned.velocity.east_m_s;
    state_msg_.inertial_vel.vector.z = pos_vel_ned.velocity.down_m_s;

    // Publish the messages to ROS2
    state_pub_->publish(state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the body_vel field in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param odom A mavsdk structure which contains vehicle odometry data expressed in NED, from which
 * we only care about the body velocity
 */
void ROSNode::update_odometry(const mavsdk::Telemetry::Odometry &odom) {
    
    // Update the body velocity message
    state_msg_.body_vel.header.stamp = rclcpp::Clock().now();
    state_msg_.body_vel.twist.linear.x = odom.velocity_body.x_m_s;
    state_msg_.body_vel.twist.linear.y = odom.velocity_body.y_m_s;
    state_msg_.body_vel.twist.linear.z = odom.velocity_body.z_m_s;
    state_pub_->publish(state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the pose.orientation field in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param quat A mavsdk structure which contains a quaternion encoding the attitude of the vehicle in NED
 */
void ROSNode::update_attitude_quaternion_ned(const mavsdk::Telemetry::Quaternion &quat) {
    // Update the orientation of the vehicle in the pose message
    state_msg_.pose.header.stamp = rclcpp::Clock().now();
    state_msg_.pose.pose.orientation.w = quat.w;
    state_msg_.pose.pose.orientation.x = quat.x;
    state_msg_.pose.pose.orientation.y = quat.y;
    state_msg_.pose.pose.orientation.z = quat.z;
    state_pub_->publish(state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the body_vel.twist field in the state_msg. This method 
 * publishes the most up to date message to state_pub
 * @param ang_vel A mavsdk structure which contains the angular velocity of the vehicle expressed in the body frame
 * according to the f.r.d frame
 */
void ROSNode::update_angular_velocity_body(const mavsdk::Telemetry::AngularVelocityBody &ang_vel) {
    state_msg_.body_vel.header.stamp = rclcpp::Clock().now();
    state_msg_.body_vel.twist.angular.x = ang_vel.roll_rad_s;
    state_msg_.body_vel.twist.angular.y = ang_vel.pitch_rad_s;
    state_msg_.body_vel.twist.angular.z = ang_vel.yaw_rad_s;
    state_pub_->publish(state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the rpy field in the state_msg. This method
 * publishes the most up to date message to state_pub
 * @param euler A mavsdk structure which contains the euler angles representing the attitude of the vehicle
 * with a body frame defined according to f.r.d relative to a NED inertial frame, expressed in the inertial frame, 
 * according to a Z-Y-X rotation
 */
void ROSNode::update_attitude_ned_euler(const mavsdk::Telemetry::EulerAngle &euler) {
    state_msg_.rpy.header.stamp = rclcpp::Clock().now();
    state_msg_.rpy.vector.x = euler.roll_deg;
    state_msg_.rpy.vector.y = euler.pitch_deg;
    state_msg_.rpy.vector.z = euler.yaw_deg;

    state_pub_->publish(state_msg_);
}

/**
 * @ingroup publisherMessageUpdate
 * @brief Method that is called to update the imu_msg and mag_msg_. This method publishes
 * the most up to date message to imu_pub and mag_pub.
 * @param imu A mavsdk structure which contains:
 * - the angular velocity measured by the imu (in rad)
 * - the linear acceleration measured by the imu (in m/s^2)
 * - the magnetic field (in gauss)
 */
void ROSNode::update_imu_state(const mavsdk::Telemetry::Imu &imu) {
    imu_msg_.header.stamp = rclcpp::Clock().now();

    // TODO - put the orientation in quaternion somewhere here

    // Angular velocity measured directly by the IMU
    imu_msg_.angular_velocity.x = imu.angular_velocity_frd.forward_rad_s;
    imu_msg_.angular_velocity.y = imu.angular_velocity_frd.right_rad_s;
    imu_msg_.angular_velocity.z = imu.angular_velocity_frd.down_rad_s;

    // Linear acceleration measured directly by the IMU
    imu_msg_.linear_acceleration.x = imu.acceleration_frd.forward_m_s2;
    imu_msg_.linear_acceleration.y = imu.acceleration_frd.right_m_s2;
    imu_msg_.linear_acceleration.z = imu.acceleration_frd.down_m_s2;
    imu_pub_->publish(imu_msg_);

    // TODO - Check if this magnetic field (received in Gauss) is being correctly scaled to Tesla Units
    mag_msg_.header.stamp = imu_msg_.header.stamp;
    mag_msg_.magnetic_field.x = imu.magnetic_field_frd.forward_gauss * 0.0001;
    mag_msg_.magnetic_field.y = imu.magnetic_field_frd.right_gauss * 0.0001;
    mag_msg_.magnetic_field.z = imu.magnetic_field_frd.down_gauss * 0.0001;
    mag_pub_->publish(mag_msg_);
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
 * @param response The response in this service goes empty, as the request is done asynchronously through mavlink
 */
void ROSNode::arm_callback(const pegasus_msgs::srv::Arm::Request::SharedPtr request, const pegasus_msgs::srv::Arm::Response::SharedPtr response) {
    mavlink_node_->arm_disarm(request->arm);
    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @ingroup servicesCallbacks
 * @brief Autoland service callback. When a service request is reached from the land_service_,
 * this callback is called and will send a mavlink command for the vehicle to autoland using the onboard controller
 * @param request An empty request for landing the vehicle (can be ignored)
 * @param response The response in this service goes empty, as the request id done asynchronously through mavlink
 */
void ROSNode::land_callback(const pegasus_msgs::srv::Land::Request::SharedPtr request, const pegasus_msgs::srv::Land::Response::SharedPtr response) {
    mavlink_node_->land();
    (void) *request; // do nothing with the empty request and avoid compilation warnings from unused argument
    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @ingroup servicesCallbacks
 * @brief Reboot service callback. When a service request is reached from the reboot_service_,
 * this callback will send a mavlink command for the vehicle's microcontroller to reboot (not the onboard PC!)
 * @param request An empty request for rebooting the vehicle (can be ignored)
 * @param response The response in this service goes empty, as the request id done asynchronously through mavlink
 */
void ROSNode::reboot_callback(const pegasus_msgs::srv::Reboot::Request::SharedPtr request, const pegasus_msgs::srv::Reboot::Response::SharedPtr response) {
    mavlink_node_->reboot();
    (void) *request; // do nothing with the empty request and avoid compilation warnings from unused argument
    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @ingroup servicesCallbacks
 * @brief Service that is avaialable for any node to request the current thrust curve of the active drone
 * as well as the mass of the vehicle (for control purposes)
 * @param request An empty request (can be ignored)
 * @param respopnse The response in this empty contains a string with the identifier of the type of thrust curve
 * used for the computations (Quadratic, Linear, etc.), the thrust curve parameter names, the actual parameter values and the mass of the vehicle
 */
void ROSNode::thrust_curve_callback(const pegasus_msgs::srv::ThrustCurve::Request::SharedPtr request, const pegasus_msgs::srv::ThrustCurve::Response::SharedPtr response) {
    (void) *request; // do nothing with the empty request and avoid compilation warnings from unused argument

    RCLCPP_INFO_STREAM(nh_->get_logger(), "Request received for vehicle mass and thrust curve");

    // Send the mass of the vehicle
    response->mass = mass_;

    // Send the type of thrust curve
    response->identifier = (thrust_curve_) ? thrust_curve_->get_type() : "None";

    // Initiate the vector of parameters
    response->parameter_names = std::vector<std::string>();
    response->parameters = std::vector<double>();

    // Update the vectors with the thrust curve parameters
    if(thrust_curve_) {
        for (auto const& [key, val] : thrust_curve_->get_parameters()) {
            response->parameter_names.push_back(key);
            response->parameters.push_back(val);
        }
    }
}

