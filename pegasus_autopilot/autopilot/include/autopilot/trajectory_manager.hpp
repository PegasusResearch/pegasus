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
#include <optional>
#include <Eigen/Core>

#include "state.hpp"
#include "trajectory.hpp"

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Custom ROS2 messages and services
#include "pegasus_msgs/srv/reset_path.hpp"

namespace autopilot {

class TrajectoryManager {

public:

    using SharedPtr = std::shared_ptr<TrajectoryManager>;
    using UniquePtr = std::unique_ptr<TrajectoryManager>;
    using WeakPtr = std::weak_ptr<TrajectoryManager>;

    // Configuration for the trajectory manager
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle      
        std::function<VehicleStatus()> get_vehicle_status;                      // Function pointer to get the current status of the vehicle  
        std::function<VehicleConstants()> get_vehicle_constants;                // Function pointer to get the current dynamical constants of the vehicle    
    };

    
    inline void initialize_trajectory_manager(const TrajectoryManager::Config& config) {
        
        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        get_vehicle_status = config.get_vehicle_status;
        get_vehicle_constants = config.get_vehicle_constants;

        // Initialize the derived class
        initialize();
    }

    virtual void initialize() = 0;
    
    virtual std::optional<Eigen::Vector3d> pd(const double & gamma) const {
        throw std::runtime_error("pd() not implemented in TrajectoryManager");
    }

    virtual std::optional<Eigen::Vector3d> d_pd(const double & gamma) const {
        throw std::runtime_error("d_pd() not implemented in TrajectoryManager");
    }

    virtual std::optional<Eigen::Vector3d> d2_pd(const double & gamma) const {
        throw std::runtime_error("d2_pd() not implemented in TrajectoryManager");
    }

    virtual std::optional<Eigen::Vector3d> d3_pd(const double & gamma) const {
        throw std::runtime_error("d3_pd() not implemented in TrajectoryManager");
    }

    virtual std::optional<Eigen::Vector3d> d4_pd(const double & gamma) const {
        throw std::runtime_error("d4_pd() not implemented in TrajectoryManager");
    }
  
    virtual std::optional<double> vd(const double & gamma) const {
        throw std::runtime_error("vd() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> d_vd(const double & gamma) const {
        throw std::runtime_error("d_vd() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> d2_vd(const double & gamma) const {
        throw std::runtime_error("d2_vd() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> curvature(const double & gamma) const {
        throw std::runtime_error("curvature() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> torsion(const double & gamma) const {
        throw std::runtime_error("torsion() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> tangent_angle(const double & gamma) const {
        throw std::runtime_error("tangent_angle() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> derivate_norm(const double & gamma) const {
        throw std::runtime_error("derivate_norm() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> min_gamma() const {
        throw std::runtime_error("min_gamma() not implemented in TrajectoryManager");
    }

    virtual std::optional<double> max_gamma() const {
        throw std::runtime_error("max_gamma() not implemented in TrajectoryManager");
    }

protected:

    // The ROS2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Function pointer to get the current state of the vehicle
    std::function<State()> get_vehicle_state{nullptr};
    std::function<VehicleStatus()> get_vehicle_status{nullptr};
    std::function<VehicleConstants()> get_vehicle_constants{nullptr};
};

} // namespace autopilot