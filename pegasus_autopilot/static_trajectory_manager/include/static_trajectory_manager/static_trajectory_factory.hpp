/****************************************************************************
 *
 *   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Pegasus nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "static_trajectory.hpp"

namespace autopilot {
    
// A factory to instantiate a trajectory object
class StaticTrajectoryFactory {

public:

    using SharedPtr = std::shared_ptr<StaticTrajectoryFactory>;
    using UniquePtr = std::unique_ptr<StaticTrajectoryFactory>;
    using WeakPtr = std::weak_ptr<StaticTrajectoryFactory>;

    // Configuration for the trajectory factory
    struct Config {
        rclcpp::Node::SharedPtr node;                                                 // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<void(StaticTrajectory::SharedPtr)> add_trajectory_to_manager;   // Method that when called with a trajectory adds it to the trajectory server
    };

    // Method that must be implemented by the derived classes
    virtual void initialize() = 0;

    // Method called internally by the trajectory server to initialize the factory
    void initialize_factory(const StaticTrajectoryFactory::Config & config) {

        // Save the node
        node_ = config.node;

        // Save the method to add a trajectory to the trajectory server
        add_trajectory_to_manager = config.add_trajectory_to_manager;

        // Perform class specific initialization        
        initialize();
    }

protected:

    // The ROS 2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Method that when called with a trajectory adds it to the trajectory server
    std::function<void(StaticTrajectory::SharedPtr)> add_trajectory_to_manager{nullptr};
};

} // namespace autopilot