/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this 
 * software must display the following acknowledgement: This product 
 * includes software developed by Project Pegasus.
 * 4. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived 
 * from this software without specific prior written permission.
 *
 * Additional Restrictions:
 * 4. The Software shall be used for non-commercial purposes only. 
 * This includes, but is not limited to, academic research, personal 
 * projects, and non-profit organizations. Any commercial use of the 
 * Software is strictly prohibited without prior written permission 
 * from the copyright holders.
 * 5. The Software shall not be used, directly or indirectly, for 
 * military purposes, including but not limited to the development 
 * of weapons, military simulations, or any other military applications. 
 * Any military use of the Software is strictly prohibited without 
 * prior written permission from the copyright holders.
 * 6. The Software may be utilized for academic research purposes, 
 * with the condition that proper acknowledgment is given in all 
 * corresponding publications.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#include "estimator_manager/estimator_manager.hpp"

namespace estimator {

EstimatorManager::EstimatorManager() : Node("pegasus_estimator_manager") {}

void EstimatorManager::initialize() {

    // Initialize the estimators
    initialize_estimators();

    // Initialize the ROS2 interface
    initialize_publishers();
    initialize_services();
}

void EstimatorManager::initialize_estimators() {

    // Read the list of operation modes from the parameter server
    this->declare_parameter<std::vector<std::string>>("estimator_manager.estimators", std::vector<std::string>());

    // Load the base class that defines the interface for all the estimators
    estimator_loader_ = std::make_unique<pluginlib::ClassLoader<estimator::Estimator>>("estimator", "estimator::Estimator");

    // Log all the estimators that are going to be loaded dynamically
    for (const std::string & estimator : this->get_parameter("estimator_manager.estimators").as_string_array()) {
        
        // Log the estimator that is about to be loaded
        RCLCPP_INFO(this->get_logger(), "Loading estimator: %s", estimator.c_str());

        // Attempt to load the estimator
        try {
            // Load the estimator and initialize it
            estimators_[estimator] = estimator::Estimator::UniquePtr(estimator_loader_->createUnmanagedInstance("estimator::" + estimator));
            estimators_[estimator]->initialize();
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading estimator: " << e.what() << ". Estimator: " << estimator);
            std::exit(EXIT_FAILURE);
        }
    }
}

void EstimatorManager::update() {

    // Initiaalize the last time (for the first time :D)
    static rclcpp::Time last_time_ = this->get_clock()->now();

    // Get the current time and compute the time difference
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    // Execute the loop for all the estimators
    // TODO: replace this by a thread pool to make it more efficient
    for (const auto& estimator : estimators_) {
        estimator.second->update(dt);
    }

    // Update the last time
    last_time_ = now;

    // Get the state from the main estimator
    State state = estimators_.at(main_estimator_)->get_state();

    // Publish the current state of the vehicle
    publish_state(state);
}

void EstimatorManager::reset(const std::string &estimator_name) {

    // Check if the estimator exists
    if (!estimators_.contains(estimator_name)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Estimator: " << estimator_name << " does not exist. Cannot reset.");
        return;
    }

    // Reset the estimator
    estimators_.at(estimator_name)->reset();
}

void EstimatorManager::publish_state(const estimator::State & state) const {

    // Message containing the state of the vehicle
    static nav_msgs::msg::Odometry state_msg_;

    // Update the state of the vehicle
    state_msg_.header.stamp = rclcpp::Clock().now();
    state_msg_.pose.pose.position.x = state.position(0);
    state_msg_.pose.pose.position.y = state.position(1);
    state_msg_.pose.pose.position.z = state.position(2);

    state_msg_.twist.twist.linear.x = state.velocity(0);
    state_msg_.twist.twist.linear.y = state.velocity(1);
    state_msg_.twist.twist.linear.z = state.velocity(2);

    state_msg_.pose.pose.orientation.w = state.attitude.w();
    state_msg_.pose.pose.orientation.x = state.attitude.x();
    state_msg_.pose.pose.orientation.y = state.attitude.y();
    state_msg_.pose.pose.orientation.z = state.attitude.z();

    state_msg_.twist.twist.angular.x = state.angular_velocity(0);
    state_msg_.twist.twist.angular.y = state.angular_velocity(1);
    state_msg_.twist.twist.angular.z = state.angular_velocity(2);

    // Publish the state of the vehicle
    state_publisher_->publish(state_msg_);
}

void EstimatorManager::initialize_publishers() {
    
    // Initialize the publisher for the state of the vehicle
    this->declare_parameter<std::string>("estimator.publishers.state", "state");
    state_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        this->get_parameter("estimator.publishers.state").as_string(), rclcpp::SensorDataQoS());
}

void EstimatorManager::initialize_services() {

    // Initialize the service to reset a given estimator
    this->declare_parameter<std::string>("estimator.services.reset", "reset");
    //reset_service_ = this->create_service<pegasus_msgs::srv::ResetEstimator>(
    //    this->get_parameter("estimator.services.reset").as_string(), std::bind(&EstimatorManager::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
}

} // namespace estimator