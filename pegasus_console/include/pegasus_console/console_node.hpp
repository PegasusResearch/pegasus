#pragma once

#include <thread>
#include <memory>
#include "console_ui.hpp"
#include "rclcpp/rclcpp.hpp"

// ROS2 messages supported
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/status.hpp"
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"

// ROS2 services supported
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/srv/offboard.hpp"
#include "pegasus_msgs/srv/kill_switch.hpp"
#include "pegasus_msgs/srv/position_hold.hpp"

// ROS2 services for the autopilot functionality
#include "pegasus_msgs/srv/set_mode.hpp"

// ROS2 services for the trajectory generations
#include "pegasus_msgs/srv/add_arc.hpp"
#include "pegasus_msgs/srv/add_line.hpp"
#include "pegasus_msgs/srv/add_circle.hpp"
#include "pegasus_msgs/srv/add_lemniscate.hpp"

class ConsoleNode : public rclcpp::Node {

public:
    
    ConsoleNode();
    ~ConsoleNode();

    void initialize_subscribers();
    void initialize_publishers();
    void initialize_services();
    
    // Basic low level operation of the vehicle
    void on_arm_disarm_click(bool arm);
    void on_land_click();
    void on_hold_click();
    void on_offboard_click();
    void on_kill_switch_click();

    // Thrust curve control of the vehicle
    void on_thrust_curve_click();
    void on_thrust_curve_stop();
    bool is_thrust_curve_running();

    // Offboard position control of the vehicle
    void on_setpoint_click();
    void on_setpoint_stop();
    bool is_setpoint_running();

    // Set the autopilot operating mode
    void on_set_autopilot_mode(const std::string & mode);

    // Add trajectory segments to the autopilot to follow
    void on_add_waypoint_click();
    void on_add_arc_click();
    void on_add_line_click();
    void on_add_circle_click();
    void on_add_lemniscate_click();

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

    // Auxiliar variables for using the thrust curve feature
    bool thrust_curve_mode_{false};
    pegasus_msgs::msg::ControlAttitude thrust_curve_msg_;
    std::thread thrust_curve_thread_;

    // Auxiliar variables for using the setpoint feature
    bool setpoint_mode_{false};
    pegasus_msgs::msg::ControlPosition setpoint_msg_;
    std::thread setpoint_thread_;

    // ROS2 subscribers
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filter_sub_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_pub_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlPosition>::SharedPtr position_pub_;

    // ROS2 service clients
    rclcpp::Client<pegasus_msgs::srv::Arm>::SharedPtr arm_disarm_client_;
    rclcpp::Client<pegasus_msgs::srv::Land>::SharedPtr land_client_;
    rclcpp::Client<pegasus_msgs::srv::KillSwitch>::SharedPtr kill_switch_client_;
    rclcpp::Client<pegasus_msgs::srv::PositionHold>::SharedPtr position_hold_client_;
    rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedPtr offboard_client_;

    // ROS2 service clients for the autopilot
    rclcpp::Client<pegasus_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    // ROS2 service clients for the trajectory generation
    rclcpp::Client<pegasus_msgs::srv::AddArc>::SharedPtr add_arc_client_;
    rclcpp::Client<pegasus_msgs::srv::AddLine>::SharedPtr add_line_client_;
    rclcpp::Client<pegasus_msgs::srv::AddCircle>::SharedPtr add_circle_client_;
    rclcpp::Client<pegasus_msgs::srv::AddLemniscate>::SharedPtr add_lemniscate_client_;
};