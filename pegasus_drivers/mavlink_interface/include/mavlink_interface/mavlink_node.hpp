#pragma once
#include <mavsdk/mavsdk.h>
#include "rclcpp/rclcpp.hpp"


class MAVLinkNode : public rclcpp::Node {
    public:

        /**
         * @brief Construct a new MAVLinkNode object
         */
        MAVLinkNode();

        /**
         * @brief Destroy the MAVLinkNode object
         */
        ~MAVLinkNode();
    private:

        /**
         * @brief Callback that is called periodically by ROS2 timer
         */
        void timer_callback();

        /**
         * @brief A MavSDK object
         */
        mavsdk::Mavsdk mavsdk_;

        /**
         * @brief Get the system object
         * 
         * @return std::shared_ptr<mavsdk::System> 
         */
        std::shared_ptr<mavsdk::System> get_system();

        /**
         * @brief A timer that periodically sends the data received by mavlink to ROS2
         */
        rclcpp::TimerBase::SharedPtr timer_;
        
};