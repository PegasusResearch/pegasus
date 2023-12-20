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

#include <map>
#include <memory>
#include <optional>
#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Custom service to reset the trajectory
#include "pegasus_msgs/srv/reset_path.hpp"

// Base class import for defining a trajectory manager
#include <autopilot/trajectory_manager.hpp>

// Definition of the static trajectories interface
#include "static_trajectory.hpp"
#include "static_trajectory_factory.hpp"

namespace autopilot {

// A StaticTrajectoryManager is a TrajectoryManager that can load static trajectories
// The static trajectories are should be derived from the StaticTrajectory class
// and are loaded on the fly using the pluginlib library
class StaticTrajectoryManager : public autopilot::TrajectoryManager {

public:

    using SharedPtr = std::shared_ptr<StaticTrajectoryManager>;
    using UniquePtr = std::unique_ptr<StaticTrajectoryManager>;
    using WeakPtr = std::weak_ptr<StaticTrajectoryManager>;

    virtual void initialize() override;

    virtual std::optional<Eigen::Vector3d> pd(const double gamma) const override;
    virtual std::optional<Eigen::Vector3d> d_pd(const double gamma) const override;
    virtual std::optional<Eigen::Vector3d> d2_pd(const double gamma) const override;
    virtual std::optional<Eigen::Vector3d> d3_pd(const double gamma) const override;
    virtual std::optional<Eigen::Vector3d> d4_pd(const double gamma) const override;

    virtual std::optional<double> vehicle_speed(const double gamma) const override;
    virtual std::optional<double> vd(const double gamma) const override;
    virtual std::optional<double> d_vd(const double gamma) const override;
    virtual std::optional<double> d2_vd(const double gamma) const override;

    virtual std::optional<double> min_gamma() const override;
    virtual std::optional<double> max_gamma() const override;

    // API to add a trajectory to the trajectory manager
    void add_trajectory(StaticTrajectory::SharedPtr trajectory);
    void reset_trajectory();
    std::optional<unsigned int> get_trajectory_index(const double gamma) const;

protected:

    // Initialize the services that reset the path, etc.
    void initialize_services();

    // Callback to handle a trajectory reset request
    void reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr request, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response);

    // Static trajectories that can be loaded into the trajectory manager
    std::map<std::string, StaticTrajectoryFactory::UniquePtr> trajectory_factories_;

    // Configurations for the trajectory factories
    StaticTrajectoryFactory::Config trajectory_config_;

    // Service to reset the current trajectory
    rclcpp::Service<pegasus_msgs::srv::ResetPath>::SharedPtr reset_trajectory_service_{nullptr};

    // Definition of the actual trajectories
    std::vector<StaticTrajectory::SharedPtr> trajectories_;
};

} // namespace autopilot