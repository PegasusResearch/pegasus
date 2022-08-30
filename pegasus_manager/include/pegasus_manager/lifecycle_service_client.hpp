#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class LifeCycleServiceClient {

public:

    /**
     * @brief Construct a new Lifecycle Service Client object
     * @param lifecycle_node_name TODO
     * @param parent_node TODO
     */
    LifecycleServiceClient(const std::string & lifecycle_node_name, rclcpp::Node::SharedPtr parent_node);

    /// Trigger a state change
    /**
    * Throws std::runtime_error on failure
    */
    bool change_state(const uint8_t transition, const std::chrono::seconds timeout);

    /// Trigger a state change, returning result
    bool change_state(std::uint8_t transition);

    /// Get the current state as a lifecycle_msgs::msg::State id value
    /**
     * Throws std::runtime_error on failure
     */
    uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

private:

    rclcpp::Node::SharedPtr node_;
    ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
    ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};