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

#include <map>
#include <vector>
#include <Eigen/Core>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"

// Plugin libraries
#include <pluginlib/class_loader.hpp>

// ROS 2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/status.hpp"
#include "pegasus_msgs/msg/vehicle_constants.hpp"
#include "pegasus_msgs/msg/autopilot_status.hpp"

// ROS 2 services
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/offboard.hpp"
#include "pegasus_msgs/srv/set_mode.hpp"

// Auxiliary libraries
#include "mode.hpp"
#include "state.hpp"
#include "geofencing.hpp"
#include "controller.hpp"
#include "trajectory_manager.hpp"

namespace autopilot {

class Autopilot : public rclcpp::Node {

public:

    Autopilot();
    ~Autopilot() {}

    // Function that executes periodically the control loop of each operation mode
    virtual void update();
    
    // Function that establishes the state machine to transition between operating modes
    virtual bool change_mode(const std::string new_mode, bool force=false);

    // Function that signals that the current mode has finished its operation and should transition to the fallback mode
    virtual void signal_mode_finished();

    // Returns the current mode of operation of the autopilot and state of the vehicle
    inline std::string get_mode() const { return current_mode_; }
    inline State get_state() const { return state_; }
    inline VehicleStatus get_status() const { return status_; }
    inline VehicleConstants get_vehicle_constants() const { return vehicle_constants_; }

    // Initializes the autopilot to run
    void initialize();

private:

    // Pre-initializations of the autopilot
    void initialize_controller();
    void initialize_geofencing();
    void initialize_trajectory_manager();
    void initialize_operating_modes();

    // ROS2 node initializations
    void initialize_autopilot();
    void initialize_publishers();
    void initialize_subscribers();
    void initialize_services();

    // Subscriber callbacks to get the current state of the vehicle
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg);
    void vehicle_constants_callback(const pegasus_msgs::msg::VehicleConstants::ConstSharedPtr msg);

    // Services callbacks to set the operation mode of the autopilot
    void change_mode_callback(const std::shared_ptr<pegasus_msgs::srv::SetMode::Request> request, std::shared_ptr<pegasus_msgs::srv::SetMode::Response> response);

    // ROS 2 service to change the operation mode
    rclcpp::Service<pegasus_msgs::srv::SetMode>::SharedPtr change_mode_service_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::AutopilotStatus>::SharedPtr status_publisher_;
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscriber_;
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_subscriber_;
    rclcpp::Subscription<pegasus_msgs::msg::VehicleConstants>::SharedPtr vehicle_constants_subscriber_;

    // ROS2 messages
    pegasus_msgs::msg::AutopilotStatus status_msg_;
    pegasus_msgs::msg::VehicleConstants vehicle_constants_msg_;

    // ROS 2 timer to handle the control modes, update the controllers and publish the control commands
    rclcpp::TimerBase::SharedPtr timer_;

    // Modes of operation of the autopilot
    std::map<std::string, autopilot::Mode::UniquePtr> operating_modes_;
    std::map<std::string, std::vector<std::string>> valid_transitions_;
    std::map<std::string, std::string> fallback_modes_;
    std::map<std::string, std::string> on_finish_modes_;
    std::map<std::string, std::string> geofencing_violation_fallback_;

    // Configuration for the operation modes for the autopilot
    Mode::Config mode_config_;

    // Current state and status of the vehicle
    State state_;
    VehicleStatus status_;
    VehicleConstants vehicle_constants_;
    std::string current_mode_{"Uninitialized"};

    // Low level controllers for reference tracking
    Controller::Config controller_config_;
    autopilot::Controller::SharedPtr controller_{nullptr};

    // Geofencing object to check if the vehicle is inside the geofence
    Geofencing::Config geofencing_config_;
    autopilot::Geofencing::UniquePtr geofencing_{nullptr};

    // Trajectory manager to handle complex trajectories and motion planning
    TrajectoryManager::Config trajectory_manager_config_;
    autopilot::TrajectoryManager::SharedPtr trajectory_manager_{nullptr};

    // Auxiliar counter to keep track when forcing a mode change
    int force_change_counter_{0};

    // Auxiliar variable used to keep track of time
    rclcpp::Time last_time_;

    // Auxiliar flag to check if the operating mode has finished
    bool mode_finished_{false};

    // Class loaders for the plugins
    std::unique_ptr<pluginlib::ClassLoader<autopilot::Mode>> mode_loader_;
    std::unique_ptr<pluginlib::ClassLoader<autopilot::Controller>> controller_loader_;
    std::unique_ptr<pluginlib::ClassLoader<autopilot::Geofencing>> geofencing_loader_;
    std::unique_ptr<pluginlib::ClassLoader<autopilot::TrajectoryManager>> trajectory_manager_loader_;
};

} // namespace autopilot