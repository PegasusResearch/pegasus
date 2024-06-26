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

#include <memory>
#include <functional>
#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Pegasus imports
#include "state.hpp"
#include "controller.hpp"
#include "trajectory_manager.hpp"

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
        std::function<void()> signal_mode_finished;                             // Function pointer to signal that the mode has finished operating
        Controller::SharedPtr controller;                                       // Controller to be used by the mode
        TrajectoryManager::SharedPtr trajectory_manager;                        // Trajectory manager that can be used by some modes
    };

    // Custom constructor like function - as we must have the default constructor for the pluginlib
    inline void initialize_mode(const Mode::Config & config) {

        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        get_vehicle_status = config.get_vehicle_status;
        get_vehicle_constants = config.get_vehicle_constants;        
        signal_mode_finished = config.signal_mode_finished;

        // Initialize the controller
        controller_ = config.controller;

        // Initialize the trajectory manager
        trajectory_manager_ = config.trajectory_manager;

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

    // The controller object that can be used by each mode to track references
    Controller::SharedPtr controller_{nullptr};

    // The trajectory manager object that can be used by each mode to generate references to track
    TrajectoryManager::SharedPtr trajectory_manager_{nullptr};

    // Function pointer which will be instantiated with the function pointers passed in the configuration
    std::function<State()> get_vehicle_state{nullptr};
    std::function<VehicleStatus()> get_vehicle_status{nullptr};
    std::function<VehicleConstants()> get_vehicle_constants{nullptr};

    // Function pointer to signal that the mode has finished operating
    std::function<void()> signal_mode_finished{nullptr};
};

} // namespace autopilot