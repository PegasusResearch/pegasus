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

    /**
     * @brief This function returns the desired position of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired position of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d pd(const double gamma) const override;

    /**
     * @brief This function returns the desired velocity of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired velocity of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d_pd(const double gamma) const override;

    /**
     * @brief This function returns the desired acceleration of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired acceleration of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d2_pd(const double gamma) const override;

    /**
     * @brief This function returns the desired jerk of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d3_pd(const double gamma) const override;

    /**
     * @brief This function returns the desired snap of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired snap of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d4_pd(const double gamma) const override;

    /**
     * @brief This function returns the vehicle speed in m/s at any given position in the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired speed of the vehicle at a given time (double)
     */
    virtual double vehicle_speed(const double gamma) const override;

    /**
     * @brief This function returns the desired vehicle speed in the trajectory frame
     * (Note that this is not expressed in m/s as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired speed of the vehicle at a given time (double)
     */
    virtual double vd(const double gamma) const override;

    /**
     * @brief This function returns the desired vehicle acceleration in the trajectory frame
     * (Note that this is not expressed in m/s^2 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired acceleration of the vehicle at a given time (double)
     */
    virtual double d_vd(const double gamma) const override;

    /**
     * @brief This function returns the desired vehicle jerk in the trajectory frame
     * (Note that this is not expressed in m/s^3 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (double)
     */
    virtual double d2_vd(const double gamma) const override;

    /**
     * @brief This function returns the minimum value of the trajectory parameter gamma
     * @return The minimum value of the trajectory parameter gamma (double)
     */
    double min_gamma() const override { return 0.0; }

    /**
     * @brief This function returns the maximum value of the trajectory parameter gamma
     * @return The maximum value of the trajectory parameter gamma (double)
     */
    double max_gamma() const override { return trajectory_max_values_.back(); }

    /**
     * @brief This functions returns whether the trajectory is empty or not
     * @return True if the trajectory is empty, false otherwise
     */
    bool empty() const override { return trajectories_.empty(); }

    /**
     * @brief This function adds a trajectory to the trajectory manager
     * vector of trajectories
     */
    inline void add_trajectory(StaticTrajectory::SharedPtr trajectory) {
        trajectories_.push_back(trajectory); 
        trajectory_max_values_.push_back(trajectory_max_values_.back() + trajectory->max_gamma());
    }

protected:

    // Initialize the services that reset the path, etc.
    void initialize_services();

    // Reset the trajectory, i.e. empty the vector of trajectories
    inline void reset_trajectory() { 
        trajectories_.clear(); 
        trajectory_max_values_.clear(); 
    }

    // Get the index of the trajectory that is currently being followed in the vector of trajectories
    int get_trajectory_index(const double gamma) const;

    // Get the index of the trajectory that is currently being followed in the vector of trajectories
    // using a binary search with Olog(n) complexity, where n is the number of trajectories in the vector
    int binary_search(double gamma, int left, int right) const;

    // Normalize the parameter gamma to the range [0,max_gamma] for the trajectory with index "index"
    double normalize_parameter(double gamma, int index) const;

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

    // Vector of the accumulated trajectory parametric lengths (used for the trajectory indexing)
    std::vector<double> trajectory_max_values_;
};

} // namespace autopilot