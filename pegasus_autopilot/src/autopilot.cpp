#include "autopilot.hpp"

namespace pegasus {

Autopilot::Autopilot() : rclcpp::Node("autopilot") {
    
    // Initialize the ROS2 interface
    initialize_parameters();
    initialize_publishers();
    initialize_subscribers();
    initialize_services();
}


void Autopilot::initialize_parameters() {

}

void Autopilot::initialize_publishers() {

}

void Autopilot::initialize_subscribers() {

    // Subscribe to the state of the vehicle
    this->declare_parameter<std::string>("subscribers.state", "state");
    rclcpp::Parameter state_topic = this->get_parameter("subscribers.state");
    state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&ROSNode::state_callback, this, std::placeholders::_1));
}

void Autopilot::initialize_services() {

    // Service to land the vehicle
    this->declare_parameter<std::string>("services.land", "land");
    rclcpp::Parameter land_topic = this->get_parameter("services.land");
    land_service_ = this->create_service<pegasus_msgs::srv::Land>(
        land_topic.as_string(), std::bind(&ROSNode::land_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Service to takeoff the vehicle up to a given hight
    this->declare_parameter<std::string>("services.takeoff", "takeoff");
    rclcpp::Parameter takeoff_topic = this->get_parameter("services.takeoff");
    takeoff_service_ = this->create_service<pegasus_msgs::srv::Takeoff>(
        takeoff_topic.as_string(), std::bind(&ROSNode::takeoff_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Service to hold the vehicle in the current position
    this->declare_parameter<std::string>("services.hold", "hold");
    rclcpp::Parameter hold_topic = this->get_parameter("services.hold");
    hold_service_ = this->create_service<pegasus_msgs::srv::Hold>(
        hold_topic.as_string(), std::bind(&ROSNode::hold_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Service to go to a given waypoint
    this->declare_parameter<std::string>("services.waypoint", "waypoint");
    rclcpp::Parameter waypoint_topic = this->get_parameter("services.waypoint");
    waypoint_service_ = this->create_service<pegasus_msgs::srv::Waypoint>(
        waypoint_topic.as_string(), std::bind(&ROSNode::waypoint_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Service to enter the passthrough mode
    this->declare_parameter<std::string>("services.pass_through", "pass_through");
    rclcpp::Parameter pass_through_topic = this->get_parameter("services.pass_through");
    pass_through_service_ = this->create_service<pegasus_msgs::srv::PassThrough>(
        pass_through_topic.as_string(), std::bind(&ROSNode::pass_through_service_callback, this, std::placeholders::_1, std::placeholders::_2));
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