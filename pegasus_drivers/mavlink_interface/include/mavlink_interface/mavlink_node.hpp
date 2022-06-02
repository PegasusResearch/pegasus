#pragma once
#include "rclcpp/rclcpp.hpp"

class MAVLinkNode : public rclcpp::Node {
    public:
        MAVLinkNode();
        ~MAVLinkNode();
    private:
        void timer_callback();

        /**
         * @brief A timer that periodically sends the data received by mavlink to ROS2
         */
        rclcpp::TimerBase::SharedPtr timer_;
        
};