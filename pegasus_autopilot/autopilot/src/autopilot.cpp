#include "autopilot/autopilot.hpp"
#include "autopilot/mode.hpp"
#include <pluginlib/class_loader.hpp>

namespace autopilot {

Autopilot::Autopilot() : Node("pegasus_autopilot") {}

void Autopilot::initialize() {
    
    // Initialize the ROS2 interface
    initialize_parameters();
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    // Initialize the timer running at the control rate defined in the configuration file
    this->declare_parameter<double>("autopilot.rate", 50.0);
    double rate = this->get_parameter("autopilot.rate").as_double();

    last_time_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&Autopilot::update, this));
    RCLCPP_INFO(this->get_logger(), "Autopilot is initialized and will run at %.2f Hz", rate);
}

void Autopilot::initialize_parameters() {

    // Read the list of operation modes from the parameter server
    this->declare_parameter<std::vector<std::string>>("autopilot.modes", std::vector<std::string>());
    rclcpp::Parameter modes = this->get_parameter("autopilot.modes");

    // Load the base class that defines the interface for all the operation modes
    pluginlib::ClassLoader<autopilot::Mode> mode_loader("autopilot", "autopilot::Mode");

    // Setup the operation mode configurations
    mode_config_.node = this->shared_from_this();
    mode_config_.get_vehicle_state = std::bind(&Autopilot::get_state, this);
    mode_config_.get_vehicle_status = std::bind(&Autopilot::get_status, this);
    mode_config_.set_position = std::bind(&Autopilot::set_target_position, this, std::placeholders::_1, std::placeholders::_2);
    mode_config_.set_attitude = std::bind(&Autopilot::set_target_attitude, this, std::placeholders::_1, std::placeholders::_2);
    mode_config_.set_attitude_rate = std::bind(&Autopilot::set_target_attitude_rate, this, std::placeholders::_1, std::placeholders::_2);
    mode_config_.signal_mode_finished = std::bind(&Autopilot::signal_mode_finished, this);

    // Log all the modes that are to be loaded dynamically
    for (const std::string & mode : modes.as_string_array()) {
        
        // Log the mode that is about to be loaded
        RCLCPP_INFO(this->get_logger(), "Loading mode: %s", mode.c_str());

        // Attempt to load the mode
        try {
            // Load the mode and initialize it
            operating_modes_[mode] = autopilot::Mode::UniquePtr(mode_loader.createUnmanagedInstance("autopilot::" + mode));

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

        } catch (const std::exception & e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Exception while loading mode: " << e.what() << ". Mode: " << mode);
        }
    }

    // Set the default/initial operating mode
    this->declare_parameter<std::string>("autopilot.default_mode", "DisarmMode");
    rclcpp::Parameter default_mode = this->get_parameter("autopilot.default_mode");
    current_mode_ = default_mode.as_string();

    // Log the default mode
    RCLCPP_INFO(this->get_logger(), "Default mode: %s", current_mode_.c_str());
}

void Autopilot::initialize_publishers() {

    // Initialize the publisher for the position control commands
    this->declare_parameter<std::string>("autopilot.publishers.control_position", "control_position");
    rclcpp::Parameter control_position_topic = this->get_parameter("autopilot.publishers.control_position");
    position_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlPosition>(
        control_position_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude control commands
    this->declare_parameter<std::string>("autopilot.publishers.control_attitude", "control_attitude");
    rclcpp::Parameter control_attitude_topic = this->get_parameter("autopilot.publishers.control_attitude");
    attitude_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the attitude rate control commands
    this->declare_parameter<std::string>("autopilot.publishers.control_attitude_rate", "control_attitude_rate");
    rclcpp::Parameter control_attitude_rate_topic = this->get_parameter("autopilot.publishers.control_attitude_rate");
    attitude_rate_publisher_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        control_attitude_rate_topic.as_string(), rclcpp::SensorDataQoS());

    // Initialize the publisher for the status of the vehicle
    this->declare_parameter<std::string>("autopilot.publishers.status", "autopilot/status");
    rclcpp::Parameter status_topic = this->get_parameter("autopilot.publishers.status");
    status_publisher_ = this->create_publisher<pegasus_msgs::msg::AutopilotStatus>(
        status_topic.as_string(), rclcpp::SensorDataQoS());
}

void Autopilot::initialize_subscribers() {

    // Subscribe to the state of the vehicle
    this->declare_parameter<std::string>("autopilot.subscribers.state", "state");
    rclcpp::Parameter state_topic = this->get_parameter("autopilot.subscribers.state");
    state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::state_callback, this, std::placeholders::_1));

    // Subscribe to the status of the vehicle
    this->declare_parameter<std::string>("autopilot.subscribers.status", "status");
    rclcpp::Parameter status_topic = this->get_parameter("autopilot.subscribers.status");
    status_subscriber_ = this->create_subscription<pegasus_msgs::msg::Status>(
        status_topic.as_string(), rclcpp::SensorDataQoS(), std::bind(&Autopilot::status_callback, this, std::placeholders::_1));
}

void Autopilot::initialize_services() {
    
    // Initialize the Service server to change the operation mode
    this->declare_parameter<std::string>("autopilot.services.set_mode", "set_mode");
    rclcpp::Parameter change_mode_service = this->get_parameter("autopilot.services.set_mode");

    change_mode_service_ = this->create_service<pegasus_msgs::srv::SetMode>(
        change_mode_service.as_string(), std::bind(&Autopilot::change_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
}

// Function that executes periodically the control loop of each operation mode
void Autopilot::update() {

    // Get the current time and compute the time difference
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    // Execute the control loop of the current mode
    try {
        operating_modes_.at(current_mode_)->update(dt);
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

    // Attemp to enter the new mode
    try {
        operating_modes_[new_mode]->enter();
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

    // Log that the current mode has finished and we are transitioning to the next mode
    RCLCPP_WARN_STREAM(this->get_logger(), "Mode: " << current_mode_ << " has finished its operation. Transitioning to mode: " << on_finish_modes_[current_mode_]);

    // Signal that the current mode has finished its operation and should transition to the fallback mode
    if (on_finish_modes_[current_mode_] != "") change_mode(on_finish_modes_[current_mode_]);
}

void Autopilot::set_target_position(const Eigen::Vector3d & position, float yaw) {

    // Set the position control message
    position_msg_.position[0] = position[0];
    position_msg_.position[1] = position[1];
    position_msg_.position[2] = position[2];
    position_msg_.yaw = yaw;

    // Publish the position control message for the controller to track
    position_publisher_->publish(position_msg_);
}

void Autopilot::set_target_attitude(const Eigen::Vector3d & attitude, float thrust_force) {

    // Set the attitude control message
    attitude_msg_.attitude[0] = attitude[0];
    attitude_msg_.attitude[1] = attitude[1];
    attitude_msg_.attitude[2] = attitude[2];
    attitude_msg_.thrust = thrust_force;

    // Publish the attitude control message for the controller to track
    attitude_publisher_->publish(attitude_msg_);
}

void Autopilot::set_target_attitude_rate(const Eigen::Vector3d & attitude_rate, float thrust_force) {

    // Set the attitude rate control message
    attitude_rate_msg_.attitude[0] = attitude_rate[0];
    attitude_rate_msg_.attitude[1] = attitude_rate[1];
    attitude_rate_msg_.attitude[2] = attitude_rate[2];
    attitude_rate_msg_.thrust = thrust_force;

    // Publish the attitude rate control message for the controller to track
    attitude_rate_publisher_->publish(attitude_rate_msg_);
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

    // Check if the vehicle is disarmed and the current mode is not armed mode or disarmed mode - if so, force a transition to disarmed mode
    // TODO - improve this logic
    if (!status_.armed && current_mode_ != "DisarmMode") {
        RCLCPP_WARN(this->get_logger(), "Vehicle is disarmed by FMU. Autopilot forcing a transition to DisarmMode");
        change_mode("DisarmMode", true);
    }

    // Check if the vehicle is ON_AIR, armed and in offboard mode. If so, it means something has died and we reconnected. In this case we should transition
    // to HoldMode and try to prevent the vehicle from crashing
    // TODO - improve this logic later on
    if (status_.flying && status_.armed && status_.offboard && current_mode_ == "DisarmMode") {
        RCLCPP_WARN(this->get_logger(), "Vehicle is ON_AIR, armed and in offboard mode. Autopilot forcing a transition to HoldMode");
        change_mode("HoldMode", true);
    }

    // TODO - if statsus is armed and not in offboard mode and current_mode == DIsarmedMode, transition to ArmedMode and status_.flying==False
}

} // namespace autopilot