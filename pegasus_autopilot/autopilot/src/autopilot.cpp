#include "autopilot/autopilot.hpp"
#include "autopilot/mode.hpp"
#include <pluginlib/class_loader.hpp>

namespace autopilot {

Autopilot::Autopilot() : Node("pegasus_autopilot") {

    // Initialize the ROS2 interface
    initialize_parameters();
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    // Initialize the timer running at 50 Hz
    last_time_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 50.0), std::bind(&Autopilot::update, this));
}

void Autopilot::initialize_parameters() {

    // Read the list of operation modes from the parameter server
    this->declare_parameter<std::vector<std::string>>("autopilot.modes", std::vector<std::string>());
    rclcpp::Parameter modes = this->get_parameter("autopilot.modes");
    
    // Load the base class that defines the interface for all the operation modes
    pluginlib::ClassLoader<autopilot::Mode> mode_loader("autopilot", "autopilot::Mode");

    std::vector<std::string> operating_modes = mode_loader.getDeclaredClasses();

    for (const std::string & mode : operating_modes) {
        RCLCPP_INFO(this->get_logger(), "Found mode: %s", mode.c_str());
    }

    std::shared_ptr<autopilot::Mode> arm = mode_loader.createSharedInstance("autopilot::ArmMode");
    arm->initialize();

    // TODO: Finish this implementation
}

void Autopilot::initialize_publishers() {

    // Initialize the publisher for the position control commands
    this->declare_parameter<std::string>("publishers.control_position", "control_position");
    rclcpp::Parameter control_position_topic = this->get_parameter("publishers.control_position");
    position_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlPosition>(
        control_position_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude control commands
    this->declare_parameter<std::string>("publishers.control_attitude", "control_attitude");
    rclcpp::Parameter control_attitude_topic = this->get_parameter("publishers.control_attitude");
    attitude_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude rate control commands
    this->declare_parameter<std::string>("publishers.control_attitude_rate", "control_attitude_rate");
    rclcpp::Parameter control_attitude_rate_topic = this->get_parameter("publishers.control_attitude_rate");
    attitude_rate_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_rate_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the status of the vehicle
    this->declare_parameter<std::string>("publishers.status", "autopilot/status");
    rclcpp::Parameter status_topic = this->get_parameter("publishers.status");
    status_publisher_ = this->create_publisher<pegasus_msgs::msg::AutopilotStatus>(
        status_topic.as_string(), rclcpp::SensorDataQoS());
}

void Autopilot::initialize_subscribers() {

    // Subscribe to the state of the vehicle
    this->declare_parameter<std::string>("subscribers.state", "state");
    rclcpp::Parameter state_topic = this->get_parameter("subscribers.state");
    state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::state_callback, this, std::placeholders::_1));

    // Subscribe to the status of the vehicle
    this->declare_parameter<std::string>("subscribers.status", "status");
    rclcpp::Parameter status_topic = this->get_parameter("subscribers.status");
    status_subscriber_ = this->create_subscription<pegasus_msgs::msg::Status>(
        status_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::status_callback, this, std::placeholders::_1));
}

void Autopilot::initialize_services() {
    // TODO
}

// Function that executes periodically the control loop of each operation mode
void Autopilot::update() {

    // Get the current time and compute the time difference
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    // Execute the control loop of the current mode
    try {
        operating_modes_.at(current_mode_)->update(dt);
        //TODO
    } catch (const std::exception & e) {
        auto steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Exception while executing update: " << e.what() << ". Mode: " << current_mode_);
    }

    // Update the last time
    last_time_ = now;

    // Publish the current status of the autopilot
    status_msg_.header.stamp = now;
    status_publisher_->publish(status_msg_);
}

// Function that establishes the state machine to transition between operating modes
void Autopilot::change_mode(const std::string new_mode) {
    // TODO - implement the state machine and the transition logic here

    // Update the current mode in the status message
    status_msg_.mode = current_mode_;
}

void Autopilot::set_target_position(const Eigen::Vector3d & position, float yaw) {

    // Set the position control message
    position_msg_.position[0] = position[0];
    position_msg_.position[1] = position[1];
    position_msg_.position[2] = position[2];
    position_msg_.yaw = yaw;

    // Publish the position control message for the controller to track
    position_publisher_->publish(position_msg_);
}

void Autopilot::set_target_attitude(const Eigen::Vector3d & attitude, float thrust_force) {

    // Set the attitude control message
    attitude_msg_.attitude[0] = attitude[0];
    attitude_msg_.attitude[1] = attitude[1];
    attitude_msg_.attitude[2] = attitude[2];
    attitude_msg_.thrust = thrust_force;

    // Publish the attitude control message for the controller to track
    attitude_publisher_->publish(attitude_msg_);
}

void Autopilot::set_target_attitude_rate(const Eigen::Vector3d & attitude_rate, float thrust_force) {

    // Set the attitude rate control message
    attitude_rate_msg_.attitude[0] = attitude_rate[0];
    attitude_rate_msg_.attitude[1] = attitude_rate[1];
    attitude_rate_msg_.attitude[2] = attitude_rate[2];
    attitude_rate_msg_.thrust = thrust_force;

    // Publish the attitude rate control message for the controller to track
    attitude_rate_publisher_->publish(attitude_rate_msg_);
}


void Autopilot::state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    // Update the state of the vehicle
    state_.position[0] = msg->pose.pose.position.x;
    state_.position[1] = msg->pose.pose.position.y;
    state_.position[2] = msg->pose.pose.position.z;

    state_.velocity[0] = msg->twist.twist.linear.x;
    state_.velocity[1] = msg->twist.twist.linear.y;
    state_.velocity[2] = msg->twist.twist.linear.z;

    state_.attitude.w() = msg->pose.pose.orientation.w;
    state_.attitude.x() = msg->pose.pose.orientation.x;
    state_.attitude.y() = msg->pose.pose.orientation.y;
    state_.attitude.z() = msg->pose.pose.orientation.z;

    state_.angular_velocity[0] = msg->twist.twist.angular.x;
    state_.angular_velocity[1] = msg->twist.twist.angular.y;
    state_.angular_velocity[2] = msg->twist.twist.angular.z;
}

void Autopilot::status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg) {

    // Update the operation status of the vehicle
    status_.armed = msg->armed;
    status_.flying = (msg->landed_state == pegasus_msgs::msg::Status::IN_AIR) ? true : false;
    status_.offboard = (msg->flight_mode == pegasus_msgs::msg::Status::OFFBOARD) ? true : false;
}

} // namespace autopilot