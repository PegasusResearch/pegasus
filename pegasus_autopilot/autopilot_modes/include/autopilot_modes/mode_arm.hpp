#pragma once

#include "rclcpp/rclcpp.hpp"
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/offboard.hpp"

#include <autopilot/mode.hpp>


namespace autopilot {

class ArmMode : public autopilot::Mode {

public:

    ~ArmMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:


    bool arm();
    bool offboard();
    void send_no_thrust_commands();

    // ROS2 service clients
    rclcpp::Client<pegasus_msgs::srv::Arm>::SharedPtr arm_client_;
    rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedPtr offboard_client_;

    // Create a callback group such that the service callbacks are executed in a separate thread
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}