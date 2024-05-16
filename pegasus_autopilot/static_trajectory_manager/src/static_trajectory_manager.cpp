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

    // Initialize the services that reset the trajectory, etc.
    initialize_services();
}

// Initialize the services that reset the path, etc.
void StaticTrajectoryManager::initialize_services() {

    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.services.reset_trajectory", "trajectory/reset_trajectory");
    
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


int StaticTrajectoryManager::get_trajectory_index(const double gamma) const {

    // If the vector of trajectories is empty, return -1
    if(trajectories_.empty()) return -1;

    // If the gamma is smaller than the minimum gamma of the first trajectory, return the index of the first trajectory
    if(gamma < 0) return 0;

    // If the gamma is larger than the maximum gamma of the last trajectory, return the index of the last trajectory
    if(gamma > trajectory_max_values_.back()) return trajectories_.size() - 1;
    
    // Get the index of the trajectory that is currently being followed in the vector of trajectories
    return binary_search(gamma, 0, trajectories_.size() - 1);
}

int StaticTrajectoryManager::binary_search(double gamma, int left, int right) const {
    
    // Compute the middle index
    int mid = left + (right - left) / 2;

    // Get the minimum value for the index mid
    double min_mid = mid == 0 ? 0 : trajectory_max_values_[mid - 1];

    // Get the maximum value for the index mid
    double max_mid = trajectory_max_values_[mid];

    // Apply the binary search algorithm
    if(gamma >= min_mid && gamma <= max_mid) return mid;
    else if(gamma > max_mid) return binary_search(gamma, mid + 1, right);
    else return binary_search(gamma, left, mid - 1);
}

double StaticTrajectoryManager::normalize_parameter(double gamma, int index) const {

    // If the index is 0, then the gamma is already normalized, 
    // otherwise subtract the maximum value of the previous trajectory
    return (index == 0) ? gamma : gamma - trajectory_max_values_[index - 1];
}

Eigen::Vector3d StaticTrajectoryManager::pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return Eigen::Vector3d::Zero();

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);

    // Return the position of the trajectory
    return trajectories_[index]->pd(normalized_gamma);
}

Eigen::Vector3d StaticTrajectoryManager::d_pd(const double gamma) const {
    
    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return Eigen::Vector3d::Zero();

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);

    return trajectories_[index]->d_pd(normalized_gamma);
}

Eigen::Vector3d StaticTrajectoryManager::d2_pd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return Eigen::Vector3d::Zero();

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->d2_pd(normalized_gamma);
}

Eigen::Vector3d StaticTrajectoryManager::d3_pd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return Eigen::Vector3d::Zero();

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->d3_pd(normalized_gamma);
}

Eigen::Vector3d StaticTrajectoryManager::d4_pd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return Eigen::Vector3d::Zero();

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);

    return trajectories_[index]->d4_pd(normalized_gamma);
}


double StaticTrajectoryManager::yaw(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->yaw(normalized_gamma);
}


double StaticTrajectoryManager::d_yaw(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->d_yaw(normalized_gamma);
}

double StaticTrajectoryManager::vehicle_speed(double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->vehicle_speed(normalized_gamma);
}

double StaticTrajectoryManager::vd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);
    
    return trajectories_[index]->vd(normalized_gamma);
}

double StaticTrajectoryManager::d_vd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);

    return trajectories_[index]->d_vd(normalized_gamma);
}

double StaticTrajectoryManager::d2_vd(const double gamma) const {

    // Get the index of the section in the sections vector
    int index = get_trajectory_index(gamma);

    // Safety check
    if (index == -1) return 0.0;

    // Make the gamma vary between 0 and max for a given trajectory section
    double normalized_gamma = normalize_parameter(gamma, index);

    return trajectories_[index]->d2_vd(normalized_gamma);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::StaticTrajectoryManager, autopilot::TrajectoryManager)