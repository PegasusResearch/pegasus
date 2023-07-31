#pragma once

#include <thread>
#include <memory>
#include "console_ui.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pegasus_msgs/msg/status.hpp"

class ConsoleNode : public rclcpp::Node {

public:
    
    ConsoleNode();
    ~ConsoleNode();

    void initialize_subscribers();
    void initialize_services();

    void start();

    // The console UI object
    ConsoleUI console_ui_;

protected:

    // ROS 2 thread setup
    std::thread executor_thread_;
    rclcpp::executors::MultiThreadedExecutor executor_;

    // ROS2 subscribers
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_sub_;
};