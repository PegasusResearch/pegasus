/*
 * Copyright 2022 Marcelo Jacinto.
 *
 * This file is part of the pegasus package and subject to the license terms
 * in the top-level LICENSE file of the pegasus repository.
 */
/**
 * @brief MAVLink Node
 * @file mavlink_node.cpp
 * @author Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 */

#include <memory>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/log_callback.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "mavlink_node.hpp"

MAVLinkNode::MAVLinkNode() : Node("mavlink_node") {

    // Define the logging scheme for mavlink library
    mavsdk::log::subscribe([](mavsdk::log::Level level,   // message severity level
                          const std::string& message,     // message text
                          const std::string& file,        // source file from which the message was sent
                          int line) {                     // line number in the source file
        
        // process the log message in a way you like
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "[" << file << " - line " << line << "]: " << message);

        // returning true from the callback disables printing the message to stdout
        return level < mavsdk::log::Level::Warn;
    });

    // Connect to a given port at a given address
    this->declare_parameter<std::string>("connection", "tcp://:14550");
    rclcpp::Parameter connection_address = this->get_parameter("connection");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Connection: " << connection_address.as_string());
    mavsdk::ConnectionResult connection_result = this->mavsdk_.add_any_connection(connection_address.as_string(), mavsdk::ForwardingOption::ForwardingOn);

    std::cerr << "Connection failed: " << connection_result << '\n';

    // Get a pointer to the current vehicle or just terminate the program
    this->system_ = this->get_system();
    std::cout << "Vehicle detected" << std::endl;

    // Instantiate the mavsdk plugins for the vehicle
    this->action_ = std::make_unique<mavsdk::Action>(this->system_);        // Enable simple actions such as arming, taking off, and landing.
    this->offboard_ = std::make_unique<mavsdk::Offboard>(this->system_);    // Control a drone with position, velocity, attitude or motor commands.
    this->telemetry_ = std::make_unique<mavsdk::Telemetry>(this->system_);  // (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates.
    this->mocap_ = std::make_unique<mavsdk::Mocap>(this->system_);          // Send mocap data to the autopilot to fuse inside the onboard filter

    // Update the Status message with the drone ID
    this->status_msg_.system_id = this->system_->get_system_id();

    // Enable forwarding to QGroundControl
    this->mavsdk_.add_any_connection("udp://127.0.0.1:14550", mavsdk::ForwardingOption::ForwardingOn);

    // Initialize the ROS2 Publishers
    this->initializeROSPublishers();

    // Initialize MAVLink Offboard, Telemetry and Actions
    this->initializeMAVLinkSubscribers();

    // Initialize ROS2 services and subscribers
    this->initializeROSSevices();
    this->initializeROSActions();
    this->initializeROSMocapSubscribers();
    this->initializeROSSubscribers();
}

MAVLinkNode::~MAVLinkNode() {
    // TODO
}

/**************************************
 * ROS2 Setup
 **************************************/
/**
 * @brief Method that is used to initialize all the ROS publishers (initialize state publishers, sensors publishers, among others)
 */
void MAVLinkNode::initializeROSPublishers() {
    
    // Initialize the publisher for the status of the vehicle (arm/disarm state, connection, ...)
    this->declare_parameter("publishers.status", "status");
    rclcpp::Parameter status_topic = this->get_parameter("publishers.status");
    this->status_pub_ = this->create_publisher<pegasus_msgs::msg::Status>(status_topic.as_string(), 1);

    // Initialize the publisher for the current state of the vehicle (position, orientation, body and inertial frame velocity)
    this->declare_parameter<std::string>("publishers.nav.state", "nav/state");
    rclcpp::Parameter state_topic = this->get_parameter("publishers.nav.state");
    this->state_pub_ = this->create_publisher<pegasus_msgs::msg::State>(state_topic.as_string(), 1);

    // Initialize the publisher for the IMU driver data
    this->declare_parameter<std::string>("publishers.sensors.imu.vel_and_accel", "sensors/imu/vel_and_accel");
    rclcpp::Parameter imu_vel_accel_topic = this->get_parameter("publishers.sensors.imu.vel_and_accel");
    this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_vel_accel_topic.as_string(), 1);
    
    this->declare_parameter<std::string>("publishers.sensors.imu.magnetic_field", "sensors/imu/magnetic_field");
    rclcpp::Parameter imu_magnetic_field_topic = this->get_parameter("publishers.sensors.imu.magnetic_field");
    this->magnetic_field_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(imu_magnetic_field_topic.as_string(), 1);

    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Publishers all initialized");
}

/**
 * @brief Get the system object from mavsdk once the vehicle is detected
 * 
 * @return std::shared_ptr<mavsdk::System> A shared pointer to the Vehicle System object
 */
std::shared_ptr<mavsdk::System> MAVLinkNode::get_system() {
    
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    this->mavsdk_.subscribe_on_new_system([&prom, this]() {
        auto system = this->mavsdk_.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            this->mavsdk_.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 5 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char ** argv) {

    // Initiate ROS2 library
    rclcpp::init(argc, argv);

    // Create a node to send and receive data between MAVLink and ROS2
    rclcpp::spin(std::make_shared<MAVLinkNode>());
    
    rclcpp::shutdown();
    return 0;
}