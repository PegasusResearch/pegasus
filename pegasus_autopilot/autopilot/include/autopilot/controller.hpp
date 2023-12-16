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
#include <stdexcept>
#include <functional>

#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Pegasus imports
#include "state.hpp"

namespace autopilot {

class Controller {

public: 

    using SharedPtr = std::shared_ptr<Controller>;
    using UniquePtr = std::unique_ptr<Controller>;
    using WeakPtr = std::weak_ptr<Controller>;

    // Configuration for the operation mode
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle      
        std::function<VehicleStatus()> get_vehicle_status;                      // Function pointer to get the current status of the vehicle  
        std::function<VehicleConstants()> get_vehicle_constants;                // Function pointer to get the current dynamical constants of the vehicle    
    };

    inline void initialize_controller(const Controller::Config & config) {

        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        get_vehicle_status = config.get_vehicle_status;
        get_vehicle_constants = config.get_vehicle_constants;

        // Initialize the derived class
        initialize();
    }

    /**
     * @brief Pure virtual function that must be implemented by the derived class to initialize the controller
    */
    virtual void initialize() = 0;

    /** 
     * @brief Sets the target position of the vehicle in the inertial frame and the target yaw (in degres)
     * @param position The target position in the inertial frame
     * @param yaw The target yaw in degrees
    */
    virtual void set_position(const Eigen::Vector3d& position, float yaw) {
        throw std::runtime_error("set_position() not implemented in derived class");
    }

    /**
     * @brief Sets the target velocity of the vehicle in the inertial frame and the target yaw rate (in degres/s)
     * @param velocity The target velocity in the inertial frame
     * @param yaw_rate The target yaw rate in degrees/s
    */
    virtual void set_velocity(const Eigen::Vector3d& velocity, float yaw_rate) {
        throw std::runtime_error("set_velocity() not implemented in derived class");
    }

    /**
     * @brief Sets the target position of the vehicle in the body frame (rotated to be vertically aligned with NED) and the target yaw (in degres)
     * @param position The target position in the body frame
     * @param yaw The target yaw in degrees
    */
    virtual void set_body_velocity(const Eigen::Vector3d& velocity, float yaw_rate) {
        throw std::runtime_error("set_body_velocity() not implemented in derived class");
    }

    /**
     * @brief Sets the target attitude of the vehicle in the body frame and the target thrust force
     * @param attitude The target attitude in the body frame (in degrees)
     * @param thrust_force The target thrust force (in Newton)
    */
    virtual void set_attitude(const Eigen::Vector3d& attitude, float thrust_force) {
        throw std::runtime_error("set_attitude() not implemented in derived class");
    }

    /**
     * @brief Sets the target attitude rate of the vehicle in the body frame and the target thrust force
     * @param attitude_rate The target attitude rate in the body frame (in degrees/s)
     * @param thrust_force The target thrust force (in Newton)
    */
    virtual void set_attitude_rate(const Eigen::Vector3d& attitude_rate, float thrust_force) {
        throw std::runtime_error("set_attitude_rate() not implemented in derived class");
    }

    /**
     * @brief Sets the target motor speed of the vehicle (from 0-100%)
    */
    virtual void set_motor_speed(const Eigen::VectorXd& motor_velocity) {
        throw std::runtime_error("set_motor_velocity() not implemented in derived class");
    }

protected:

    // The ROS 2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Function pointer which will be instantiated with the function pointers passed in the configuration
    std::function<State()> get_vehicle_state{nullptr};
    std::function<VehicleStatus()> get_vehicle_status{nullptr};
    std::function<VehicleConstants()> get_vehicle_constants{nullptr};
};

} // namespace autopilot