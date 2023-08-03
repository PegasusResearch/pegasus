#pragma once

// ROS Libraries
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/status.hpp"

// ROS 2 services
#include "pegasus_msgs/srv/take_off.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/srv/position_hold.hpp"
#include "pegasus_msgs/srv/waypoint.hpp"
#include "pegasus_msgs/srv/follow_trajectory.hpp"
#include "pegasus_msgs/srv/mission.hpp"
#include "pegasus_msgs/srv/pass_through.hpp"

#include <Eigen/Core>

// Library that implements the PID controller
#include "pid/pid.hpp"

namespace pegasus {

// Class that implements the base autopilot that every other autopilot should inherit from
class Autopilot : public rclcpp::Node {

public:

    // Constructor and Destructor of the autopilot
    Autopilot();
    ~Autopilot();

    // Current mode of operation of the autopilot
    enum Mode {
        DISARMED,
        ARMED_LANDED,
        TAKEOFF,
        LAND,
        HOLD,
        WAYPOINT,
        FOLLOW_TRAJECTORY,
        MISSION,
        PASS_THROUGH
    };

    // Returns the current mode of operation of the autopilot and state of the vehicle
    inline Mode get_mode() const { return mode_; }
    inline State get_state() const { return state_; }

    // Auxiliar functions that set the target position, attitude or attitude rate for the inner-loops to track
    void set_target_position(const Eigen::Vector3d & position, float yaw);
    void set_target_attitude(const Eigen::Vector3d & attitude, float thrust_force);
    void set_target_attitude_rate(const Eigen::Vector3d & attitude_rate, float thrust_force);

private:

    // ROS2 node initializations
    void initialize_parameters();
    void initialize_publishers();
    void initialize_subscribers();
    void initialize_services();

    // Subscriber callbacks to get the current state of the vehicle
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg);

    // Service callbacks to switch between operation modes
   
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscriber_;
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_subscriber_;

    // PID control for the position
    std::array<Pegasus::Pid::UniquePtr, 3> pid_control_;

    // Current mode of operation of the autopilot
    Mode mode_{Mode::STOP};

    // Current state of the vehicle
    State state_;
};

}

