#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/status.hpp"
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"
#include "pegasus_msgs/msg/autopilot_status.hpp"

// ROS 2 services
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/offboard.hpp"

// Auxiliary libraries
#include "mode.hpp"
#include "state.hpp"

namespace autopilot {

class Autopilot : public rclcpp::Node {

public:

    Autopilot();
    ~Autopilot() {}

    // Function that executes periodically the control loop of each operation mode
    virtual void update();
    
    // Function that establishes the state machine to transition between operating modes
    virtual void change_mode(const std::string new_mode);

    // Functions that set the target position, attitude or attitude rate for the inner-loops to track
    virtual void set_target_position(const Eigen::Vector3d & position, float yaw);
    virtual void set_target_attitude(const Eigen::Vector3d & attitude, float thrust_force);
    virtual void set_target_attitude_rate(const Eigen::Vector3d & attitude_rate, float thrust_force);

    // Returns the current mode of operation of the autopilot and state of the vehicle
    inline std::string get_mode() const { return current_mode_; }
    inline State get_state() const { return state_; }
    inline VehicleStatus get_status() const { return status_; }

private:

    // ROS2 node initializations
    void initialize_parameters();
    void initialize_publishers();
    void initialize_subscribers();
    void initialize_services();

    // Subscriber callbacks to get the current state of the vehicle
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg);

    // ROS2 service clients
    rclcpp::Client<pegasus_msgs::srv::Arm>::SharedPtr arm_client_;
    rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedPtr offboard_client_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlPosition>::SharedPtr position_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_publisher_;
    rclcpp::Publisher<pegasus_msgs::msg::AutopilotStatus>::SharedPtr status_publisher_;
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscriber_;
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_subscriber_;

    // ROS2 messages
    pegasus_msgs::msg::ControlPosition position_msg_;
    pegasus_msgs::msg::ControlAttitude attitude_msg_;
    pegasus_msgs::msg::ControlAttitude attitude_rate_msg_;
    pegasus_msgs::msg::AutopilotStatus status_msg_;

    // ROS 2 timer to handle the control modes, update the controllers and publish the control commands
    rclcpp::TimerBase::SharedPtr timer_;

    // Modes of operation of the autopilot
    std::map<std::string, Mode::UniquePtr> operating_modes_;
    std::map<std::string, std::vector<std::string>> valid_transitions_;

    // Current state and status of the vehicle
    State state_;
    VehicleStatus status_;
    std::string current_mode_;

    // Auxiliar variable used to keep track of time
    rclcpp::Time last_time_;
};

} // namespace autopilot