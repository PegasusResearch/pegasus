/**
 * @file manager_node.hpp
 * @author Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
 * @brief A simple manager/state machine for the pegasus nodes which inherit a lifecycle
 * node
 * @version 0.1
 * @date 2022-08-30
 * 
 * @copyright Copyright (c) 2022 Marcelo Jacinto
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may 
 * be used to endorse or promote products derived from this software without specific 
 * prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 */
#pragma once
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "ros_lifecycle_service_client/lifecycle_service_client.hpp"

class ManagerNode: public rclcpp::Node {

public:

    /**
     * @brief Construct a new Manager Node object
     * @param node_name A string with the standard name of the node
     * @param intra_process_comms Whether to use ROS2 intra-process communication feature (for low over-head communication)
     */
    ManagerNode(const std::string & node_name, bool intra_process_comms=false);

    /**
     * @brief Destroy the Manager Node object
     */
    ~ManagerNode();

    /**
     * @defgroup support_service_calls
     * This section defines all the support functions used to make the service calls
     * that will manage the lifecycle nodes
     */
    
    /**
     * @ingroup support_service_calls
     * @brief Start up managed nodes.
     * @return true or false
     */
    bool startup();

    /**
     * @brief Transition a given node to a desired target state
     * @param node A string with the name of the node
     * @param transition The target transition that enables a desired state
     * @return bool Whether the change was successfull or not
     */
    bool change_state_of_node(const std::string & node, std::uint8_t transition);

    /**
     * @brief Transition all the nodes to a desired target state
     * @param transition The target transition that enables the desired state
     * @return bool Whether the change was successfull or not
     */
    bool change_state_of_all_nodes(std::uint8_t transition);

private:

    /**
     * @brief Defines if the managed systems should be started automatically
     */
    bool autostart_{true};

    /**
     * @brief The initial timer used to setup the service clients and call the startup() function
     */
    rclcpp::TimerBase::SharedPtr init_timer_;

    /**
     * @brief A vector of node names to manage
     */
    std::vector<std::string> node_names_;

    /**
     * @brief A map of the expected transitions to primary states
     */
    std::unordered_map<std::uint8_t, std::uint8_t> transition_state_map_;

    /**
     * @brief A map of string to the transition labels
     */
    std::map<std::uint8_t, std::string> transition_label_map_;

    /**
     * @brief A map of all nodes to be controlled
     */
    
    std::map<std::string, std::shared_ptr<Pegasus::ROS::LifeCycleServiceClient>> node_map_;
};