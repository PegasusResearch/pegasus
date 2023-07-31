#pragma once

#include <thread>
#include <memory>
#include "console_ui.hpp"
#include "rclcpp/rclcpp.hpp"

// ROS2 messages supported
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/status.hpp"

// ROS2 services supported
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/srv/kill_switch.hpp"
#include "pegasus_msgs/srv/position_hold.hpp"

class ConsoleNode : public rclcpp::Node {

public:
    
    ConsoleNode();
    ~ConsoleNode();

    void initialize_subscribers();
    void initialize_services();
    
    void on_arm_disarm_click(bool arm);
    void on_land_click();
    void on_hold_click();
    void on_kill_switch_click();

    void start();

    // The console UI object
    ConsoleUI::UniquePtr console_ui_;

protected:

    // Callbacks for the ROS2 subscribers
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg);

    // Configuration for the console UI
    ConsoleUI::Config config_;

    // ROS 2 thread setup
    std::thread executor_thread_;
    rclcpp::executors::MultiThreadedExecutor executor_;

    // ROS2 subscribers
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filter_sub_;

    // ROS2 service clients
    rclcpp::Client<pegasus_msgs::srv::Arm>::SharedPtr arm_disarm_client_;
    rclcpp::Client<pegasus_msgs::srv::Land>::SharedPtr land_client_;
    rclcpp::Client<pegasus_msgs::srv::KillSwitch>::SharedPtr kill_switch_client_;
    rclcpp::Client<pegasus_msgs::srv::PositionHold>::SharedPtr position_hold_client_;
};