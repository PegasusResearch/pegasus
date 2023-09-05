#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/kill_switch.hpp"

namespace autopilot {

class DisarmMode : public autopilot::Mode {

public:

    ~DisarmMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    bool disarm();

    // ROS2 service clients
    rclcpp::Client<pegasus_msgs::srv::KillSwitch>::SharedPtr disarm_client_;

    // ----- THIS CODE WILL REPLACE THE CODE BELLOW WHEN WE SWITCH TO ROS HUMBLE IN THE VEHICLES ------
    // // Create a callback group such that the service callbacks are executed in a separate thread
    // rclcpp::CallbackGroup::SharedPtr callback_group_;
    // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // ----- THIS CODE IS ONLY USED IN ROS FOXY --------
    rclcpp::Node::SharedPtr sub_node_;
};

}