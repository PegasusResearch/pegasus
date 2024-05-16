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
#include "autopilot/autopilot.hpp"
#include "autopilot/mode.hpp"

namespace autopilot {

Autopilot::Autopilot() : Node("pegasus_autopilot") {}

void Autopilot::initialize() {
    
    // Initialize the ROS2 interface
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    // Only after the autopilot receives the vehicle constants it can initialize the operation modes and start its operation
    // This is performed in the vehicle_constants_callback function, because we assume the vehicle constants do not change over time
    // and we only need to receive them once. We must initialize the operation modes after receiving the vehicle constants, because
    // some of the operation modes depend on the vehicle constants to initialize their parameters
}

void Autopilot::initialize_autopilot() {

    // Initialize the controller
    initialize_controller();

    // Initialize geofencing
    initialize_geofencing();

    // Initialize the trajectory manager
    initialize_trajectory_manager();
    
    // Initialize the operating modes
    initialize_operating_modes();
}

void Autopilot::initialize_controller() {
    
    // Read the controller to use from the parameter server
    this->declare_parameter<std::string>("autopilot.controller", "");
    rclcpp::Parameter controller_name = this->get_parameter("autopilot.controller");

    // Load the base class that defines the interface for all the controllers
    controller_loader_ = std::make_unique<pluginlib::ClassLoader<autopilot::Controller>>("autopilot", "autopilot::Controller");

    // Setup the configurations for the controller
    controller_config_.node = this->shared_from_this();
    controller_config_.get_vehicle_state = std::bind(&Autopilot::get_state, this);
    controller_config_.get_vehicle_status = std::bind(&Autopilot::get_status, this);
    controller_config_.get_vehicle_constants = std::bind(&Autopilot::get_vehicle_constants, this);

    // Log the controller that is about to be loaded
    RCLCPP_INFO(this->get_logger(), "Loading controller: %s", controller_name.as_string().c_str());

    // Attempt to load the controller
    try {
        // Load the controller and initialize it
        controller_ = controller_loader_->createSharedInstance("autopilot::" + controller_name.as_string());
        controller_->initialize_controller(controller_config_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading controller: " << e.what() << ". Controller: " << controller_name.as_string());
        std::exit(EXIT_FAILURE);
    }
}

void Autopilot::initialize_geofencing() {

    // Read the geofencing mechanism to be used from the parameter server
    this->declare_parameter<std::string>("autopilot.geofencing", "");
    rclcpp::Parameter geofencing_name = this->get_parameter("autopilot.geofencing");

    // Load the base class that defines the interface for all the geofencing mechanisms
    geofencing_loader_ = std::make_unique<pluginlib::ClassLoader<autopilot::Geofencing>>("autopilot", "autopilot::Geofencing");

    // Check if the geofencing mechanism is set to none. If so, do not load any geofencing mechanism
    if (geofencing_name.as_string() != std::string("")) {

        // Setup the configurations for the geofencing mechanism
        geofencing_config_.node = this->shared_from_this();
        geofencing_config_.get_vehicle_state = std::bind(&Autopilot::get_state, this);
        geofencing_config_.get_vehicle_status = std::bind(&Autopilot::get_status, this);
        geofencing_config_.get_vehicle_constants = std::bind(&Autopilot::get_vehicle_constants, this);

        // Log the geofencing mechanism that is about to be loaded
        RCLCPP_INFO(this->get_logger(), "Loading geofencing mechanism: %s", geofencing_name.as_string().c_str());

        // Attempt to load the geofencing mechanism
        try {
            // Load the geofencing mechanism and initialize it
            geofencing_ = autopilot::Geofencing::UniquePtr(geofencing_loader_->createUnmanagedInstance("autopilot::" + geofencing_name.as_string()));
            geofencing_->initialize_geofencing(geofencing_config_);
        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading geofencing mechanism: " << e.what() << ". Geofencing mechanism: " << geofencing_name.as_string());
            std::exit(EXIT_FAILURE);
        }
    }
}

void Autopilot::initialize_trajectory_manager() {

    // Read the trajectory manager to use from the parameter server
    this->declare_parameter<std::string>("autopilot.trajectory_manager", "");
    rclcpp::Parameter trajectory_manager_name = this->get_parameter("autopilot.trajectory_manager");

    // Load the base class that defines the interface for all the trajectory managers
    trajectory_manager_loader_ = std::make_unique<pluginlib::ClassLoader<autopilot::TrajectoryManager>>("autopilot", "autopilot::TrajectoryManager");

    // Setup the configurations for the trajectory manager
    trajectory_manager_config_.node = this->shared_from_this();
    trajectory_manager_config_.get_vehicle_state = std::bind(&Autopilot::get_state, this);
    trajectory_manager_config_.get_vehicle_status = std::bind(&Autopilot::get_status, this);
    trajectory_manager_config_.get_vehicle_constants = std::bind(&Autopilot::get_vehicle_constants, this);

    // Log the trajectory manager that is about to be loaded
    RCLCPP_INFO(this->get_logger(), "Loading trajectory manager: %s", trajectory_manager_name.as_string().c_str());

    // Attempt to load the trajectory manager
    try {
        // Load the trajectory manager and initialize it
        trajectory_manager_ = trajectory_manager_loader_->createSharedInstance("autopilot::" + trajectory_manager_name.as_string());
        trajectory_manager_->initialize_trajectory_manager(trajectory_manager_config_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading trajectory manager: " << e.what() << ". Trajectory manager: " << trajectory_manager_name.as_string());
        std::exit(EXIT_FAILURE);
    }
}

void Autopilot::initialize_operating_modes() {

    // Read the list of operation modes from the parameter server
    this->declare_parameter<std::vector<std::string>>("autopilot.modes", std::vector<std::string>());
    rclcpp::Parameter modes = this->get_parameter("autopilot.modes");

    // Load the base class that defines the interface for all the operation modes
    mode_loader_ = std::make_unique<pluginlib::ClassLoader<autopilot::Mode>>("autopilot", "autopilot::Mode");

    // Setup the operation mode configurations
    mode_config_.node = this->shared_from_this();
    mode_config_.get_vehicle_state = std::bind(&Autopilot::get_state, this);
    mode_config_.get_vehicle_status = std::bind(&Autopilot::get_status, this);
    mode_config_.get_vehicle_constants = std::bind(&Autopilot::get_vehicle_constants, this);
    mode_config_.signal_mode_finished = std::bind(&Autopilot::signal_mode_finished, this);
    mode_config_.controller = controller_;
    mode_config_.trajectory_manager = trajectory_manager_;

    // Log all the modes that are to be loaded dynamically
    for (const std::string & mode : modes.as_string_array()) {
        
        // Log the mode that is about to be loaded
        RCLCPP_INFO(this->get_logger(), "Loading mode: %s", mode.c_str());

        // Attempt to load the mode
        try {
            // Load the mode and initialize it
            operating_modes_[mode] = autopilot::Mode::UniquePtr(mode_loader_->createUnmanagedInstance("autopilot::" + mode));

            // Initialize the mode
            operating_modes_[mode]->initialize_mode(mode_config_);

            // Load the valid transitions for this mode
            this->declare_parameter<std::vector<std::string>>("autopilot." + mode + ".valid_transitions", std::vector<std::string>());
            rclcpp::Parameter valid_transitions = this->get_parameter("autopilot." + mode + ".valid_transitions");
            valid_transitions_[mode] = valid_transitions.as_string_array();

            // Load the fallback mode for this mode
            this->declare_parameter<std::string>("autopilot." + mode + ".fallback", "");
            fallback_modes_[mode] = this->get_parameter("autopilot." + mode + ".fallback").as_string();

            // Load the on_finish mode for this mode
            this->declare_parameter<std::string>("autopilot." + mode + ".on_finish", "");
            on_finish_modes_[mode] = this->get_parameter("autopilot." + mode + ".on_finish").as_string();

            // Load the geofencing violation fallback mode for this mode
            this->declare_parameter<std::string>("autopilot." + mode + ".geofencing_violation_fallback", "");
            geofencing_violation_fallback_[mode] = this->get_parameter("autopilot." + mode + ".geofencing_violation_fallback").as_string();

        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading mode: " << e.what() << ". Mode: " << mode);
        }
    }

    // Log all the geofencing violation fallback modes
    for (const auto & [mode, fallback_mode] : geofencing_violation_fallback_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Geofencing fallback for " << mode << " is: " << fallback_mode);
    }

    // Validate that all fallback modes exist
    for (const auto & [mode, fallback_mode] : fallback_modes_) {
        if (!operating_modes_.contains(fallback_mode)) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Fallback mode: " << fallback_mode << " was not loaded. Required by " << mode);
            std::exit(EXIT_FAILURE);
        }
    }

    // Validate that all geoefncing violation fallback modes exist
    for (const auto & [mode, fallback_mode] : geofencing_violation_fallback_) {
        if (!operating_modes_.contains(fallback_mode) && fallback_mode != "") {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Geofencing fallback mode: " << fallback_mode << " was not loaded. Required by " << mode);
            std::exit(EXIT_FAILURE);
        }
    }

    // Set the default/initial operating mode
    this->declare_parameter<std::string>("autopilot.default_mode", "DisarmMode");
    rclcpp::Parameter default_mode = this->get_parameter("autopilot.default_mode");
    current_mode_ = default_mode.as_string();

    // Log the default mode
    RCLCPP_INFO(this->get_logger(), "Default mode: %s", current_mode_.c_str());

    // Initialize the timer running at the control rate defined in the configuration file
    this->declare_parameter<double>("autopilot.rate", 50.0);
    double rate = this->get_parameter("autopilot.rate").as_double();

    last_time_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Autopilot::update, this));
    RCLCPP_INFO(this->get_logger(), "Autopilot is initialized and will run at %.2f Hz", rate);
}

void Autopilot::initialize_publishers() {

    // Initialize the publisher for the status of the vehicle
    this->declare_parameter<std::string>("autopilot.publishers.status", "autopilot/status");
    status_publisher_ = this->create_publisher<pegasus_msgs::msg::AutopilotStatus>(
        this->get_parameter("autopilot.publishers.status").as_string(), rclcpp::SensorDataQoS());
}

void Autopilot::initialize_subscribers() {

    // Subscribe to the state of the vehicle
    this->declare_parameter<std::string>("autopilot.subscribers.state", "state");
    state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("autopilot.subscribers.state").as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::state_callback, this, std::placeholders::_1));

    // Subscribe to the status of the vehicle
    this->declare_parameter<std::string>("autopilot.subscribers.status", "status");
    status_subscriber_ = this->create_subscription<pegasus_msgs::msg::Status>(
        this->get_parameter("autopilot.subscribers.status").as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::status_callback, this, std::placeholders::_1));

    // Subscribe to the constants of the vehicle
    this->declare_parameter<std::string>("autopilot.subscribers.constants", "vehicle_constants");
    vehicle_constants_subscriber_ = this->create_subscription<pegasus_msgs::msg::VehicleConstants>(
        this->get_parameter("autopilot.subscribers.constants").as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::vehicle_constants_callback, this, std::placeholders::_1));
}

void Autopilot::initialize_services() {
    
    // Initialize the Service server to change the operation mode
    this->declare_parameter<std::string>("autopilot.services.set_mode", "set_mode");
    change_mode_service_ = this->create_service<pegasus_msgs::srv::SetMode>(
        this->get_parameter("autopilot.services.set_mode").as_string(), std::bind(&Autopilot::change_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// Function that executes periodically the control loop of each operation mode
void Autopilot::update() {

    // Get the current time and compute the time difference
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    // Execute the control loop of the current mode
    try {

        // Perform an update of the current mode
        operating_modes_.at(current_mode_)->update(dt);

        // Check if the mode has finished
        if (mode_finished_) {

            // Reset the mode finished flag
            mode_finished_ = false;

            // Log that the current mode has finished and we are transitioning to the next mode
            RCLCPP_WARN_STREAM(this->get_logger(), "Mode: " << current_mode_ << " has finished its operation. Transitioning to mode: " << on_finish_modes_[current_mode_]);

            // Signal that the current mode has finished its operation and should transition to the fallback mode
            if (on_finish_modes_[current_mode_] != "") change_mode(on_finish_modes_[current_mode_]);
        }

        // Check if a geofencing violation has occured. If so, and the geofencing violation fallback mode is not empty, transition to the fallback mode
        if (geofencing_ && geofencing_->check_geofencing_violation() && geofencing_violation_fallback_[current_mode_] != "") {
            
            // Log the incident
            auto steady_clock = rclcpp::Clock();
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Geofencing violation has occured. Transitioning to mode: " << geofencing_violation_fallback_[current_mode_]);
            change_mode(geofencing_violation_fallback_[current_mode_]);
        }

    } catch (const std::exception & e) {

        // If an exception is thrown during the operation mode, enter the fallback mode imediately
        change_mode(fallback_modes_[current_mode_]);

        // Log the incident for future debugging
        auto steady_clock = rclcpp::Clock();
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, "Exception while executing update: " << e.what() << ". Mode: " << current_mode_);
    }

    // Update the last time
    last_time_ = now;

    // Publish the current status of the autopilot
    status_msg_.header.stamp = now;
    status_msg_.mode = current_mode_;
    status_publisher_->publish(status_msg_);
}

// Function that establishes the state machine to transition between operating modes
bool Autopilot::change_mode(const std::string new_mode, bool force) {

    // Check if the requested mode exists in the dictionary. If not return false
    if (!operating_modes_.contains(new_mode)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Requested mode: " << new_mode << " does not exist. Keeping the operating mode: " << current_mode_);
        return false;
    } else if (new_mode == current_mode_ && !force) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Requested mode: " << new_mode << " is the same as the current mode. Keeping the operating mode: " << current_mode_);
        return false;
    }

    // Check if the request mode is a valid transition from the current mode. If not return false
    if (!std::count(valid_transitions_[current_mode_].begin(), valid_transitions_[current_mode_].end(), new_mode) && !force) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Requested mode: " << new_mode << " is not a valid transition from the current mode: " << current_mode_ << ". Keeping the operating mode: " << current_mode_);
            return false;
    }

    // Attemp to enter the new mode (but do not change the current mode yet, as we still need to exit the current mode)
    try {

        // Attempt to enter the new mode
        bool new_mode_success = operating_modes_[new_mode]->enter();

        // If the new mode was not entered successfully, return false
        if (!new_mode_success) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to enter mode: " << new_mode << ". Keeping the operating mode: " << current_mode_);
            return false;
        }

    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while entering mode: " << e.what() << ". Mode: " << new_mode << ". Keeping the operating mode: " << current_mode_);
        return false;
    }

    // Update the new operating mode
    std::string old_mode = current_mode_;
    current_mode_ = new_mode;

    // Attemp to exit the current mode
    try {
        // Reset the previous operating mode
        operating_modes_[old_mode]->exit();
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while exiting mode: " << e.what() << ". Mode: " << current_mode_ << ". Transitioned to operating mode: " << new_mode << ". Consider Landing for safety.");
    }

    // Update the current mode in the status message
    status_msg_.mode = current_mode_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Transitioned from mode: " << old_mode << " to mode: " << current_mode_);
    return true;
}

void Autopilot::signal_mode_finished() {
    
    // Set the mode finished flag to true
    mode_finished_ = true;
}

void Autopilot::change_mode_callback(const std::shared_ptr<pegasus_msgs::srv::SetMode::Request> request, std::shared_ptr<pegasus_msgs::srv::SetMode::Response> response) {
    
    // Attemp to change the mode of operation of the autopilot - Return the current mode of operation of the autopilot
    response->success = change_mode(request->mode);
}

void Autopilot::state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    // Update the state of the vehicle
    state_.position[0] = msg->pose.pose.position.x;
    state_.position[1] = msg->pose.pose.position.y;
    state_.position[2] = msg->pose.pose.position.z;

    state_.velocity[0] = msg->twist.twist.linear.x;
    state_.velocity[1] = msg->twist.twist.linear.y;
    state_.velocity[2] = msg->twist.twist.linear.z;

    state_.attitude.w() = msg->pose.pose.orientation.w;
    state_.attitude.x() = msg->pose.pose.orientation.x;
    state_.attitude.y() = msg->pose.pose.orientation.y;
    state_.attitude.z() = msg->pose.pose.orientation.z;

    state_.angular_velocity[0] = msg->twist.twist.angular.x;
    state_.angular_velocity[1] = msg->twist.twist.angular.y;
    state_.angular_velocity[2] = msg->twist.twist.angular.z;
}

void Autopilot::status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg) {

    // Update the operation status of the vehicle
    status_.armed = msg->armed;
    status_.flying = (msg->landed_state == pegasus_msgs::msg::Status::IN_AIR) ? true : false;
    status_.offboard = (msg->flight_mode == pegasus_msgs::msg::Status::OFFBOARD) ? true : false;

    // If the autopilot is not initialized yet, return
    if (current_mode_ == "Uninitialized") return;

    // Check if the vehicle is disarmed and the current mode is not armed mode or disarmed mode - if so, force a transition to disarmed mode
    // TODO - improve this logic
    if (!status_.armed && current_mode_ != "DisarmMode") {
        
        // Increment the counter for forcing a transition - this is done to prevent the autopilot from forcing a transition to DisarmMode
        force_change_counter_++;

        if (force_change_counter_ > 3) {
            RCLCPP_WARN(this->get_logger(), "Vehicle is disarmed by FMU. Autopilot forcing a transition to DisarmMode");
            change_mode("DisarmMode", true);
        }
        return;
    }

    // Check if the vehicle is ON_AIR, armed and in offboard mode. If so, it means something has died and we reconnected. In this case we should transition
    // to HoldMode and try to prevent the vehicle from crashing
    // TODO - improve this logic later on
    if (status_.flying && status_.armed && status_.offboard && current_mode_ == "DisarmMode") {

        // Increment the counter for forcing a transition - this is done to prevent the autopilot from forcing a transition to HoldMode
        force_change_counter_++;
        
        if (force_change_counter_ > 3) {
            RCLCPP_WARN(this->get_logger(), "Vehicle is ON_AIR, armed and in offboard mode. Autopilot forcing a transition to HoldMode");
            change_mode("HoldMode", true);
        }
        return;
    }

    // Reset the change counter
    force_change_counter_ = 0;

    // TODO - if status is armed and not in offboard mode and current_mode == DIsarmedMode, transition to ArmedMode and status_.flying==False
}

void Autopilot::vehicle_constants_callback(const pegasus_msgs::msg::VehicleConstants::ConstSharedPtr msg) {

    // Save the parameters of the vehicle
    vehicle_constants_.mass = msg->mass;
    vehicle_constants_.thrust_curve_id = msg->thrust_curve.identifier;
    vehicle_constants_.thurst_curve_params = msg->thrust_curve.parameters;
    vehicle_constants_.thrust_curve_values = msg->thrust_curve.values;

    // Unsubscribe from the vehicle constants topic (as we assume they do not change over time)
    vehicle_constants_subscriber_.reset();

    // Log the vehicle constants for debugging
    RCLCPP_INFO(this->get_logger(), "Vehicle constants: mass: %.2f", vehicle_constants_.mass);
    RCLCPP_INFO(this->get_logger(), "Vehicle constants: thrust_curve_id: %s", vehicle_constants_.thrust_curve_id.c_str());
    for(int i = 0; i < vehicle_constants_.thurst_curve_params.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "%s: %.4f", vehicle_constants_.thurst_curve_params[i].c_str(), vehicle_constants_.thrust_curve_values[i]);
    }

    // After getting the vehicle constants, we are ready to initialize the parameters of the operation modes
    // and let the autopilot start its operation
    initialize_autopilot();
}

} // namespace autopilot