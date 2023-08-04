#include "autopilot.hpp"

namespace Pegasus {

Autopilot::Autopilot() : rclcpp::Node("autopilot") {
    
    // Initialize the ROS2 interface
    initialize_parameters();
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    // Initialize the timer running at 50 Hz
    last_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1.0 / 50.0), std::bind(&Autopilot::update, this));
}


void Autopilot::initialize_parameters() {

    // Read the list of operation modes from the parameter server
    this->declare_parameter<std::vector<std::string>>("modes", {});
}

void Autopilot::initialize_publishers() {

    // Initialize the publisher for the position control commands
    this->declare_parameter<std::string>("publishers.control_position", "control_position");
    rclcpp::Parameter control_position_topic = this->get_parameter("publishers.control_position");
    control_position_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlPosition>(
        control_position_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude control commands
    this->declare_parameter<std::string>("publishers.control_attitude", "control_attitude");
    rclcpp::Parameter control_attitude_topic = this->get_parameter("publishers.control_attitude");
    control_attitude_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude rate control commands
    this->declare_parameter<std::string>("publishers.control_attitude_rate", "control_attitude_rate");
    rclcpp::Parameter control_attitude_rate_topic = this->get_parameter("publishers.control_attitude_rate");
    control_attitude_rate_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_rate_topic.as_string(), rclcpp::SensorDataQoS());
}

void Autopilot::initialize_subscribers() {

    // Subscribe to the state of the vehicle
    this->declare_parameter<std::string>("subscribers.state", "state");
    rclcpp::Parameter state_topic = this->get_parameter("subscribers.state");
    state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::state_callback, this, std::placeholders::_1));

    // Subscribe to the status of the vehicle
    this->declare_parameter<std::string>("subscribers.status", "status");
    rclcpp::Parameter status_topic = this->get_parameter("subscribers.status");
    status_subscriber_ = this->create_subscription<pegasus_msgs::msg::Status>(
        status_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::status_callback, this, std::placeholders::_1));
}

void Autopilot::initialize_services() {


}

// Function that executes periodically the control loop of each operation mode
void Autopilot::update() {

    // Get the current time and compute the time difference
    auto now = this->now();
    double dt = (now - last_time_).seconds();

    try {
        // Try to execute the current mode 
        mode_map_.at(mode_)->update(dt);
    }

    // Update the last time
    last_time_ = now;
}

// Function that establishes the state machine to transition between operating modes
void Autopilot::change_mode(const uint8_t new_mode) {

}


void Autopilot::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Update the state of the vehicle
    state_.position[0] = msg->pose.pose.position.x;
    state_.position[1] = msg->pose.pose.position.y;
    state_.position[2] = msg->pose.pose.position.z;

    state_.velocity[0] = msg->twist.twist.linear.x;
    state_.velocity[1] = msg->twist.twist.linear.y;
    state_.velocity[2] = msg->twist.twist.linear.z;

    state_.orientation.w = msg->pose.pose.orientation.w;
    state_.orientation.x = msg->pose.pose.orientation.x;
    state_.orientation.y = msg->pose.pose.orientation.y;
    state_.orientation.z = msg->pose.pose.orientation.z;

    state_.angular_velocity[0] = msg->twist.twist.angular.x;
    state_.angular_velocity[1] = msg->twist.twist.angular.y;
    state_.angular_velocity[2] = msg->twist.twist.angular.z;
}

void Autopilot::status_callback(const pegasus_msgs::msg::Status::SharedPtr msg) {

    // Update the operation status of the vehicle
    status_.armed = msg->armed;
    status_.flying = (msg->landed_state == pegasus_msgs::msg::Status::IN_AIR) ? true : false;
    status_.offboard = (msg->flight_mode == pegasus_msgs::msg::Status::OFFBOARD) ? true : false;
}