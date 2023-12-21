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
#include <pluginlib/class_loader.hpp>

#include "static_trajectory_manager/static_trajectory_factory.hpp"
#include "static_trajectory_manager/static_trajectory_manager.hpp"

namespace autopilot {

void StaticTrajectoryManager::initialize() {

    // Read the trajectory Factories to load from the parameter server
    node_->declare_parameter<std::vector<std::string>>("autopilot.StaticTrajectoryManager.trajectories", std::vector<std::string>());
    rclcpp::Parameter trajectories = node_->get_parameter("autopilot.StaticTrajectoryManager.trajectories");

    // Load the base class that defines the interface for all the static trajectories
    pluginlib::ClassLoader<autopilot::StaticTrajectoryFactory> trajectory_loader("autopilot", "autopilot::StaticTrajectoryFactory");

    // Setup the configurations for the trajectory factories
    trajectory_config_.node = node_;
    trajectory_config_.add_trajectory_to_manager = std::bind(&StaticTrajectoryManager::add_trajectory, this, std::placeholders::_1);

    // Log all the trajectory factories that are going to be loaded dynamically
    for(const std::string & trajectory : trajectories.as_string_array()) {
        
        // Log the trajectory factory that is about to be loaded
        RCLCPP_INFO(node_->get_logger(), "Loading trajectory: %s", trajectory.c_str());

        // Attempt to load the trajectory factory
        try {

            // Load the trajectory factory
            trajectory_factories_[trajectory] = autopilot::StaticTrajectoryFactory::UniquePtr(trajectory_loader.createUnmanagedInstance("autopilot::" + trajectory));

            // Initialize the trajectory manager
            trajectory_factories_[trajectory]->initialize_factory(trajectory_config_);

        } catch(const std::exception & ex) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Exception while loading trajectory: " << ex.what() << ". Trajectory: " << trajectory);
        }
    }
}

// Initialize the services that reset the path, etc.
void StaticTrajectoryManager::initialize_services() {

    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.services.reset_trajectory", "path/reset_trajectory");
    
    // Create the service that resets the path
    reset_trajectory_service_ = node_->create_service<pegasus_msgs::srv::ResetPath>(
        node_->get_parameter("autopilot.StaticTrajectoryManager.services.reset_trajectory").as_string(),
        std::bind(&StaticTrajectoryManager::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
}

// Callback to handle a trajectory reset request
void StaticTrajectoryManager::reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr request, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response) {

    RCLCPP_INFO_STREAM(node_->get_logger(), "Resetting trajectory.");

    // Clear the trajectory
    reset_trajectory();

    // Make the response of this service to true
    response->success = true;
    RCLCPP_INFO_STREAM(node_->get_logger(), "Trajectory empty.");
}

/**
 * @brief Add a trajectory section shared pointer to the end of the path
 * @param section A shared pointer to a generic path section
 */
void StaticTrajectoryManager::add_trajectory(StaticTrajectory::SharedPtr trajectory) {
    trajectories_.push_back(trajectory);
}

/**
 * @brief Clear the trajectory (removes all segments if any were added)
 */
void StaticTrajectoryManager::reset_trajectory() {
    // Empty the trajectories vector
    trajectories_.clear();
}

/**
 * @brief Get the index of a given section for a parametric value inside the sections vector
 * @param gamma The parametric value
 * @return unsigned int The index inside the section vector
 */
std::optional<unsigned int> StaticTrajectoryManager::get_trajectory_index(const double gamma) const {

    // For now we are assuming that each section is parameterized between 0 and 1. 
    // TODO: Change this in the future to allow for dynamic parameterizations
    unsigned int index = std::floor(gamma);
    if(index >= trajectories_.size()) index = trajectories_.size() - 1;

    return (!trajectories_.empty()) ? std::make_optional<unsigned int>(index) : std::nullopt;
}

std::optional<Eigen::Vector3d> StaticTrajectoryManager::pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
       
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<Eigen::Vector3d>(trajectories_[index.value()]->pd(normalized_gamma));
    }
    
    return std::nullopt;
}

std::optional<Eigen::Vector3d> StaticTrajectoryManager::d_pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<Eigen::Vector3d>(trajectories_[index.value()]->d_pd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<Eigen::Vector3d> StaticTrajectoryManager::d2_pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<Eigen::Vector3d>(trajectories_[index.value()]->d2_pd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<Eigen::Vector3d> StaticTrajectoryManager::d3_pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<Eigen::Vector3d>(trajectories_[index.value()]->d3_pd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<Eigen::Vector3d> StaticTrajectoryManager::d4_pd(const double gamma) const {

    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<Eigen::Vector3d>(trajectories_[index.value()]->d4_pd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<double> StaticTrajectoryManager::vehicle_speed(double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<double>(trajectories_[index.value()]->vehicle_speed(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<double> StaticTrajectoryManager::vd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<double>(trajectories_[index.value()]->vd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<double> StaticTrajectoryManager::d_vd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<double>(trajectories_[index.value()]->d_vd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<double> StaticTrajectoryManager::d2_vd(const double gamma) const {

    // Get the index of the section in the sections vector
    std::optional<unsigned int> index = get_trajectory_index(gamma);

    // If there is a section for a given gamma, then access it and return the position, otherwise return a null optional
    if(index.has_value()) {
        // Make the gamma vary between 0 and 1 for a given path section
        double normalized_gamma = gamma - static_cast<double>(index.value());

        return std::optional<double>(trajectories_[index.value()]->d2_vd(normalized_gamma));
    }

    return std::nullopt;
}

std::optional<double> StaticTrajectoryManager::min_gamma() const {
    return 0.0;
}

std::optional<double> StaticTrajectoryManager::max_gamma() const {
    return (trajectories_.empty()) ? 0.0 : (double) trajectories_.size();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::StaticTrajectoryManager, autopilot::TrajectoryManager)