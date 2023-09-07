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

#include <functional>
#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Pegasus imports
#include "state.hpp"

namespace autopilot {

class Mode {

public:

    using SharedPtr = std::shared_ptr<Mode>;
    using UniquePtr = std::unique_ptr<Mode>;
    using WeakPtr = std::weak_ptr<Mode>;

    // Configuration for the operation mode
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle      
        std::function<VehicleStatus()> get_vehicle_status;                      // Function pointer to get the current status of the vehicle  
        std::function<VehicleConstants()> get_vehicle_constants;                // Function pointer to get the current dynamical constants of the vehicle    
        std::function<void(const Eigen::Vector3d &, float)> set_position;       // Function pointer to set the position of the vehicle
        std::function<void(const Eigen::Vector3d &, float)> set_attitude;       // Function pointer to set the attitude of the vehicle
        std::function<void(const Eigen::Vector3d &, float)> set_attitude_rate;  // Function pointer to set the attitude rate of the vehicle
        std::function<void()> signal_mode_finished;                             // Function pointer to signal that the mode has finished operating
    };

    // Custom constructor like function - as we must have the default constructor for the pluginlib
    inline void initialize_mode(const Mode::Config & config) {

        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        get_vehicle_status = config.get_vehicle_status;
        get_vehicle_constants = config.get_vehicle_constants;
        set_position = config.set_position;
        set_attitude = config.set_attitude;
        set_attitude_rate = config.set_attitude_rate;
        
        signal_mode_finished = config.signal_mode_finished;

        // Initialize the derived class
        initialize();
    }

    // Methods that can be implemented by derived classes
    // that are executed by the state machine when entering, exiting or updating the mode
    virtual void initialize() = 0;
    virtual bool enter() = 0;
    virtual bool exit() = 0;
    virtual void update(double dt) = 0;

protected:

    // The ROS 2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Function pointer which will be instantiated with the function pointers passed in the configuration
    std::function<State()> get_vehicle_state{nullptr};
    std::function<VehicleStatus()> get_vehicle_status{nullptr};
    std::function<VehicleConstants()> get_vehicle_constants{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_position{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_attitude{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_attitude_rate{nullptr};

    // Function pointer to signal that the mode has finished operating
    std::function<void()> signal_mode_finished{nullptr};
};

} // namespace autopilot