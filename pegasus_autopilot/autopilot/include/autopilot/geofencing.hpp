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

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Pegasus imports 
#include "state.hpp"

namespace autopilot {

class Geofencing {

public:

    using SharedPtr = std::shared_ptr<Geofencing>;
    using UniquePtr = std::unique_ptr<Geofencing>;
    using WeakPtr = std::weak_ptr<Geofencing>;

    // Configuration for the geofencing
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle
        std::function<VehicleStatus()> get_vehicle_status;                      // Function pointer to get the current status of the vehicle
        std::function<VehicleConstants()> get_vehicle_constants;                // Function pointer to get the current constants of the vehicle
        std::function<void()> set_geofencing_violation;                         // Function pointer to signal the autopilot that a geofencing violation has occured
    };

    // Custom constructor like function - as we must have the default constructor for the pluginlib
    inline void initialize_geofencing(const Geofencing::Config & config) {
        
        // Initialialize the node
        node_ = config.node;

        // Initialize the function pointers
        get_vehicle_state_ = config.get_vehicle_state;
        get_vehicle_status_ = config.get_vehicle_status;
        get_vehicle_constants_ = config.get_vehicle_constants;

        // Perform custom initialization
        initialize();
    }

    /** @brief Initialize the custom geofencing object */
    virtual void initialize() = 0;

    /** 
     * @brief Checks if a geofencing violation has ocurred.
     * @return true if a geofencing violation has ocurred, false otherwise
     */
    virtual bool check_geofencing_violation() = 0;

protected:

    // Node
    rclcpp::Node::SharedPtr node_;

    // Function pointers
    std::function<State()> get_vehicle_state_;
    std::function<VehicleStatus()> get_vehicle_status_;
    std::function<VehicleConstants()> get_vehicle_constants_;
};

} //namespace autopilot