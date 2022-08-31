// Copyright (c) 2018 Intel Corporation
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

#include "rclcpp/rclcpp.hpp"

namespace Pegasus::ROS {

/**
 * @brief An utilitary class for quickly declaring ROS2 service clients
 * @tparam ServiceT A template of the type of ROS service
 */
template<class ServiceT>
class ServiceClient {
public:

    using SharedPtr = std::shared_ptr<ServiceClient<ServiceT>>;
    using UniquePtr = std::unique_ptr<ServiceClient<ServiceT>>;
    using WeakPtr = std::weak_ptr<ServiceClient<ServiceT>>;

    using RequestType = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;
  
    /**
     * @brief A constructor
     * @param service_name name of the service to call
     * @param provided_node Node to create the service client off of
     */
    explicit ServiceClient(const std::string & service_name, const rclcpp::Node::SharedPtr & provided_node) : service_name_(service_name), node_(provided_node) {

        // Create the service client
        client_ = node_->create_client<ServiceT>(service_name);
    }

    /**
     * @brief Invoke the service and block until completed or timed out
     * @param request The request object to call the service using
     * @param timeout Maximum timeout to wait for, default infinite
     * @return Response A pointer to the service response from the request
     * Throws std::runtime_error if the service request timesout
     */
    typename ResponseType::SharedPtr invoke(typename RequestType::SharedPtr & request, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) {
        
        // Wait for the service availability for at most 2 second
        while (!client_->wait_for_service(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error(service_name_ + " service client: interrupted while waiting for service");
            }

            // Informe the user that we are waiting for the service to be available before calling it
            RCLCPP_INFO(node_->get_logger(), "%s service client: waiting for service to appear...", service_name_.c_str());
        }

        // Send the request asynchronously
        RCLCPP_INFO(node_->get_logger(), "%s service client: send async request", service_name_.c_str());
        auto result = client_->async_send_request(request).get();

        // return the result of the service
        return result;
    }

    /**
     * @brief Invoke the service and block until completed
     * @param request The request object to call the service using
     * @param Response A pointer to the service response from the request
     * @return bool Whether it was successfully called
     * Throws std::runtime_error if the service is not available in 1 second timespan. 
     * If the service is available, this method will lock without timeout until the service result is retrieved
     */
    bool invoke(typename RequestType::SharedPtr & request, typename ResponseType::SharedPtr & response) {

        // Wait for the service availability for at most 2 second
        while (!client_->wait_for_service(std::chrono::seconds(2))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error(service_name_ + " service client: interrupted while waiting for service");
            }

            // Informe the user that we are waiting for the service to be available before calling it
            RCLCPP_INFO(node_->get_logger(), "%s service client: waiting for service to appear...", service_name_.c_str());
        }

        // Send the request asynchronously
        RCLCPP_INFO(node_->get_logger(), "%s service client: send async request", service_name_.c_str());

        // Wait until the request result is available infinitely and return the response
        response = client_->async_send_request(request).get();
        return true;
    }

    /**
     * @brief Block until a service is available or timeout
     * @param timeout Maximum timeout to wait for, default infinite
     * @return bool true if service is available
     */
    bool wait_for_service(const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) {
        return client_->wait_for_service(timeout);
    }

    /**
     * @brief Gets the service name
     * @return string Service name
     */
    std::string getServiceName() {
        return service_name_;
    }

protected:

    /**
     * @brief The service name to subscribe to
     */
    std::string service_name_;

    /**
     * @brief A shared pointer to the parent node subscribing the service
     */
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief The new exclusive callback group created for this service
     */
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    /**
     * @brief An executor for this service callback
     */
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    /**
     * @brief The actual service client
     */
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}