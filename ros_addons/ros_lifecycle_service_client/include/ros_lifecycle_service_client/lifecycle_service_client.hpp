// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ros_service_client/ros_service_client.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace Pegasus::ROS {

class LifeCycleServiceClient {

public:

    /**
     * @brief Construct a new Lifecycle Service Client object
     * @param lifecycle_node_name The name of the node to administrate
     * @param parent_node A shared pointer to the base node the service will be created from
     */
    LifeCycleServiceClient(const std::string & lifecycle_node_name, rclcpp::Node::SharedPtr parent_node);

    /**
     * @brief Trigger a state change
     * @param transition The code of the transition to trigger
     * @param timeout The number of seconds to await for the transition to finish
     * @return bool A boolean whether the transition was processed or not
     * Throws std::runtime_error on failure
    */
    bool change_state(const uint8_t transition, const std::chrono::seconds timeout);

    /**
     * @brief Trigger a state change, returning result
     * @param transition The code of the transition to trigger
     * @return bool A boolean whether the transition was processed or not
     */
    bool change_state(std::uint8_t transition);

    /**
     * @brief Get the current state as a lifecycle_msgs::msg::State id value
     * Throws std::runtime_error on failure
     */
    uint8_t get_state(const std::chrono::seconds timeout = std::chrono::seconds(2));

private:

    /**
     * @brief A shared pointer to the base node class
     */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief A service client to request a change state of a lifecycle node
     */
    Pegasus::ROS::ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;

    /**
     * @brief A service client to request the current state of a lifecycle node
     */
    Pegasus::ROS::ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}