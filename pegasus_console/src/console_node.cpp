#include "console_node.hpp"
#include "pegasus_utils/rotations.hpp"

ConsoleNode::ConsoleNode() : rclcpp::Node("pegasus_console") {

    // Initialize the subscribers, services and publishers
    initialize_subscribers();
    initialize_services();

    // Initialize the callbacks for the console UI
    config_.on_arm_disarm_click = std::bind(&ConsoleNode::on_arm_disarm_click, this, std::placeholders::_1);
    config_.on_land_click = std::bind(&ConsoleNode::on_land_click, this);
    config_.on_hold_click = std::bind(&ConsoleNode::on_hold_click, this);
    config_.on_kill_switch_click = std::bind(&ConsoleNode::on_kill_switch_click, this);

    // Initialize the console UI
    console_ui_ = std::make_unique<ConsoleUI>(config_);
}

ConsoleNode::~ConsoleNode() {}

void ConsoleNode::initialize_subscribers() {

    // Status of the vehicle
    status_sub_ = this->create_subscription<pegasus_msgs::msg::Status>(
        "/drone1/fmu/status", rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::status_callback, this, std::placeholders::_1));
    
    // Subscribe to the state of the vehicle given by its internal EKF filter
    filter_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/drone1/fmu/filter/state", 
        rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::state_callback, this, std::placeholders::_1));
}

void ConsoleNode::initialize_services() {

    // Create the service clients
    arm_disarm_client_ = this->create_client<pegasus_msgs::srv::Arm>("/drone1/fmu/arm");
    land_client_ = this->create_client<pegasus_msgs::srv::Land>("/drone1/fmu/land");
    kill_switch_client_ = this->create_client<pegasus_msgs::srv::KillSwitch>("/drone1/fmu/kill_switch");
    position_hold_client_ = this->create_client<pegasus_msgs::srv::PositionHold>("/drone1/fmu/hold");
}

void ConsoleNode::start() {

    // Add this node to the multithread executor
    executor_.add_node(this->shared_from_this());

    // Start the executor in a separate thread
    executor_thread_ = std::thread([this]() {this->executor_.spin();});

    // Start the console UI in this thread
    console_ui_->loop();
}

void ConsoleNode::on_arm_disarm_click(bool arm) {

    std::thread([this, arm]() {
        // Create and fill the request to arm or disarm the vehicle
        auto request = std::make_shared<pegasus_msgs::srv::Arm::Request>();
        request->arm = arm;

        using namespace std::chrono_literals;

        // Wait for the service to be available
        while (!arm_disarm_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        arm_disarm_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Arm>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Arm/disarm response: %s", response->success ? "true" : "false");
        });
    }).detach();
    
}

void ConsoleNode::on_land_click() {

    std::thread([this]() {

        // Create and fill the request to land the vehicle
        auto request = std::make_shared<pegasus_msgs::srv::Land::Request>();
        
        using namespace std::chrono_literals;

        // Wait for the service to be available
        while (!land_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        land_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Land>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Land response: %s", response->success ? "true" : "false");
        });
    }).detach();

}

void ConsoleNode::on_hold_click() {

    std::thread([this]() {

        // Create and fill the request to hold the vehicle
        auto request = std::make_shared<pegasus_msgs::srv::PositionHold::Request>();

        using namespace std::chrono_literals;

        // Wait for the service to be available
        while (!position_hold_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        position_hold_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::PositionHold>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Hold response: %s", response->success ? "true" : "false");
        });

    }).detach();
}

void ConsoleNode::on_kill_switch_click() {

    std::thread([this]() {

        // Create and fill the request to kill the vehicle
        auto request = std::make_shared<pegasus_msgs::srv::KillSwitch::Request>();
        request->kill = true;

        using namespace std::chrono_literals;

        // Wait for the service to be available
        while (!kill_switch_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        kill_switch_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::KillSwitch>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Kill switch response: %s", response->success ? "true" : "false");
        });

    }).detach();
}

void ConsoleNode::status_callback(const pegasus_msgs::msg::Status::ConstSharedPtr msg) {

    // Update the status of the vehicle
    console_ui_->status_.system_id = msg->system_id;
    console_ui_->status_.armed = msg->armed;
    console_ui_->status_.landed_state = msg->landed_state;
    console_ui_->status_.flight_mode = msg->flight_mode;

    // Update the battery status
    console_ui_->status_.battery.temperature = msg->battery.temperature;
    console_ui_->status_.battery.voltage = msg->battery.voltage;
    console_ui_->status_.battery.current = msg->battery.current;
    console_ui_->status_.battery.percentage = msg->battery.percentage;
    console_ui_->status_.battery.amps_hour_consumed = msg->battery.amps_hour_consumed;

    // Update the Health status
    console_ui_->status_.health.is_armable = msg->health.is_armable;
    console_ui_->status_.health.acc_calibrated = msg->health.accelerometer_calibrated;
    console_ui_->status_.health.mag_calibrated = msg->health.magnetometer_calibrated;
    console_ui_->status_.health.local_position_ok = msg->health.local_position_ok;
    console_ui_->status_.health.global_position_ok = msg->health.global_position_ok;
    console_ui_->status_.health.home_position_ok = msg->health.home_position_ok;

    // Update the RC status
    console_ui_->status_.rc_status.available = msg->rc_status.available;
    console_ui_->status_.rc_status.signal_strength = msg->rc_status.signal_strength;
}

void ConsoleNode::state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

    // Update the current position
    console_ui_->state_.position[0] = msg->pose.pose.position.x;
    console_ui_->state_.position[1] = msg->pose.pose.position.y;
    console_ui_->state_.position[2] = msg->pose.pose.position.z;

    // Update the current velocity
    console_ui_->state_.velocity_inertial[0] = msg->twist.twist.linear.x;
    console_ui_->state_.velocity_inertial[1] = msg->twist.twist.linear.y;
    console_ui_->state_.velocity_inertial[2] = msg->twist.twist.linear.z;

    // Update the current attitude
    console_ui_->state_.attitude_euler = Pegasus::Rotations::quaternion_to_euler(
        Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        )
    );

    // Convert the attitude from rad to deg
    console_ui_->state_.attitude_euler[0] = Pegasus::Rotations::rad_to_deg(console_ui_->state_.attitude_euler[0]);
    console_ui_->state_.attitude_euler[1] = Pegasus::Rotations::rad_to_deg(console_ui_->state_.attitude_euler[1]);
    console_ui_->state_.attitude_euler[2] = Pegasus::Rotations::rad_to_deg(console_ui_->state_.attitude_euler[2]);

    // Update the current angular velocity
    console_ui_->state_.angular_velocity[0] = msg->twist.twist.angular.x;
    console_ui_->state_.angular_velocity[1] = msg->twist.twist.angular.y;
    console_ui_->state_.angular_velocity[2] = msg->twist.twist.angular.z;
}