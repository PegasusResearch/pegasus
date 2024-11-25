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
#pragma once

#include <map>
#include <memory>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "nav_msgs/msg/odometry.hpp"

// Plugin libraries
#include <pluginlib/class_loader.hpp>

// Auxiliary Library
#include "state.hpp"
#include "estimator.hpp"

namespace estimator {

class EstimatorManager : public rclcpp::Node {

public:

    EstimatorManager();
    ~EstimatorManager() {}

    // Initializes the estimator node
    void initialize();

    // Function that executes periodically the loop for computing the state of the system
    // or for just computing prediction steps (for example in a Kalman Filter)
    void update();

    // Function to reset the estimators
    void reset(const std::string &estimator_name);

    // Function to set the state of the vehicle (this will only be accessible by the main estimator)
    void set_state();

    // Function to publish the state of the vehicle
    void publish_state(const estimator::State & state) const;

protected:

    // Method that loads the estimators into memory
    void initialize_estimators();

    // Initializes the publishers, subscribers and services for the statistics
    // and the main state of the vehicle
    void initialize_publishers();
    void initialize_services();

    // ROS2 publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_publisher_;

    // Map containing all the estimators that are to be used
    std::string main_estimator_;
    std::map<std::string, estimator::Estimator::UniquePtr> estimators_;

    // ROS 2 timer to handle the updates of the estimators
    rclcpp::TimerBase::SharedPtr timer_;

    // Class loaders for the plugins
    std::unique_ptr<pluginlib::ClassLoader<estimator::Estimator>> estimator_loader_;
};

}