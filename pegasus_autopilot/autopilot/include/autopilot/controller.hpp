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
     * @brief Method that can be used to reset certain parameters of the controller.
     * This method is called by the autopilot when the vehicle is switched to the disarm mode. If not implemented, it does nothing.
     */
    virtual void reset_controller() {};

    /** 
     * @brief Sets the target position of the vehicle in the inertial frame and the target yaw (in degres)
     * @param position The target position in the inertial frame
     * @param yaw The target yaw in degrees
    */
   virtual void set_position(const Eigen::Vector3d & position, double yaw, double dt) {
        set_position(position, yaw, 0.0, dt);
    }

    virtual void set_position(const Eigen::Vector3d& position, double yaw, double yaw_rate, double dt) {
        set_position(position, Eigen::Vector3d::Zero(), yaw, yaw_rate, dt);
    }

    virtual void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, double yaw, double yaw_rate, double dt) {
        set_position(position, velocity, Eigen::Vector3d::Zero(), yaw, yaw_rate, dt);
    }

    virtual void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, double yaw, double yaw_rate, double dt) {
        set_position(position, velocity, acceleration, Eigen::Vector3d::Zero(3), yaw, yaw_rate, dt);
    }

    virtual void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, double yaw, double yaw_rate, double dt) {
        set_position(position, velocity, acceleration, jerk, Eigen::Vector3d::Zero(3), yaw, yaw_rate, dt);
    }

    virtual void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate, double dt) {
        throw std::runtime_error("set_position() not implemented in derived class");
    }

    /**
     * @brief Sets the target velocity of the vehicle in the inertial frame and the target yaw rate (in degres/s)
     * @param velocity The target velocity in the inertial frame (m/s NED)
     * @param yaw_rate The target yaw in degrees
    */
    virtual void set_inertial_velocity(const Eigen::Vector3d& velocity, double yaw, double dt=0) {
        throw std::runtime_error("set_inertial_velocity() not implemented in derived class");
    }

    /**
     * @brief Sets the target position of the vehicle in the body frame (rotated to be vertically aligned with NED) and the target yaw (in degres)
     * @param position The target position in the body frame (m/s f.r.d)
     * @param yaw The target yaw-rate in degrees/s
    */
    virtual void set_body_velocity(const Eigen::Vector3d& velocity, double yaw_rate, double dt=0) {
        throw std::runtime_error("set_body_velocity() not implemented in derived class");
    }

    /**
     * @brief Set the inertial acceleration (Ax, Ay, Az) (m/s^2) of the vehicle. The adopted frame is NED 
     * @param acceleration The target acceleration in the inertial frame (NED)
     */
    virtual void set_inertial_acceleration(const Eigen::Vector3d& acceleration, double dt=0) {
        throw std::runtime_error("set_acceleration() not implemented in derived class");
    }

    /**
     * @brief Sets the target attitude of the vehicle in the body frame and the target thrust force
     * @param attitude The target attitude in the body frame (in degrees)
     * @param thrust_force The target thrust force (in Newton)
    */
    virtual void set_attitude(const Eigen::Vector3d& attitude, double thrust_force, double dt=0) {
        throw std::runtime_error("set_attitude() not implemented in derived class");
    }

    /**
     * @brief Sets the target attitude rate of the vehicle in the body frame and the target thrust force
     * @param attitude_rate The target attitude rate in the body frame (in degrees/s)
     * @param thrust_force The target thrust force (in Newton)
    */
    virtual void set_attitude_rate(const Eigen::Vector3d& attitude_rate, double thrust_force, double dt=0) {
        throw std::runtime_error("set_attitude_rate() not implemented in derived class");
    }

    /**
     * @brief Sets the target motor speed of the vehicle (from 0-100%)
    */
    virtual void set_motor_speed(const Eigen::VectorXd& motor_velocity, double dt=0) {
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