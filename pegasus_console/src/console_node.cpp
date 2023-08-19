#include "console_node.hpp"
#include "pegasus_utils/rotations.hpp"

ConsoleNode::ConsoleNode() : rclcpp::Node("pegasus_console") {

    // Initialize the subscribers, services and publishers
    initialize_publishers();
    initialize_subscribers();
    initialize_services();

    // Initialize the callbacks for the console UI
    config_.on_arm_disarm_click = std::bind(&ConsoleNode::on_arm_disarm_click, this, std::placeholders::_1);
    config_.on_land_click = std::bind(&ConsoleNode::on_land_click, this);
    config_.on_hold_click = std::bind(&ConsoleNode::on_hold_click, this);
    config_.on_offboard_click = std::bind(&ConsoleNode::on_offboard_click, this);
    config_.on_kill_switch_click = std::bind(&ConsoleNode::on_kill_switch_click, this);

    // Thrust curve control of the vehicle
    config_.on_thrust_curve_click = std::bind(&ConsoleNode::on_thrust_curve_click, this);
    config_.on_thrust_curve_stop = std::bind(&ConsoleNode::on_thrust_curve_stop, this);
    config_.is_thrust_curve_running = std::bind(&ConsoleNode::is_thrust_curve_running, this);

    // Offboard position control of the vehicle
    config_.on_setpoint_click = std::bind(&ConsoleNode::on_setpoint_click, this);
    config_.on_setpoint_stop = std::bind(&ConsoleNode::on_setpoint_stop, this);
    config_.is_setpoint_running = std::bind(&ConsoleNode::is_setpoint_running, this);

    // Autopilot mode of the vehicle
    config_.on_set_autopilot_mode = std::bind(&ConsoleNode::on_set_autopilot_mode, this, std::placeholders::_1);

    // Add trajectory segments to the autopilot to follow
    config_.on_add_waypoint_click = std::bind(&ConsoleNode::on_add_waypoint_click, this);
    config_.on_add_arc_click = std::bind(&ConsoleNode::on_add_arc_click, this);
    config_.on_add_line_click = std::bind(&ConsoleNode::on_add_line_click, this);
    config_.on_add_circle_click = std::bind(&ConsoleNode::on_add_circle_click, this);
    config_.on_add_lemniscate_click = std::bind(&ConsoleNode::on_add_lemniscate_click, this);

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

void ConsoleNode::initialize_publishers() {

    // Publish the attitude setpoints to the vehicle for thrust curve control
    attitude_rate_pub_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        "/drone1/fmu/in/throtle/attitude_rate", rclcpp::SensorDataQoS());

    // Publish the setpoints to the vehicle
    position_pub_ = this->create_publisher<pegasus_msgs::msg::ControlPosition>(
        "/drone1/fmu/in/position", rclcpp::SensorDataQoS());
}

void ConsoleNode::initialize_services() {

    // Create the service clients
    arm_disarm_client_ = this->create_client<pegasus_msgs::srv::Arm>("/drone1/fmu/arm");
    land_client_ = this->create_client<pegasus_msgs::srv::Land>("/drone1/fmu/land");
    kill_switch_client_ = this->create_client<pegasus_msgs::srv::KillSwitch>("/drone1/fmu/kill_switch");
    position_hold_client_ = this->create_client<pegasus_msgs::srv::PositionHold>("/drone1/fmu/hold");
    offboard_client_ = this->create_client<pegasus_msgs::srv::Offboard>("/drone1/fmu/offboard");

    // Create the service clients for the autopilot
    set_mode_client_ = this->create_client<pegasus_msgs::srv::SetMode>("/drone1/autopilot/set_mode");

    // Create the service clients for the trajectory generation
    add_arc_client_ = this->create_client<pegasus_msgs::srv::AddArc>("/drone1/autopilot/add_arc");
    add_line_client_ = this->create_client<pegasus_msgs::srv::AddLine>("/drone1/autopilot/add_line");
    add_circle_client_ = this->create_client<pegasus_msgs::srv::AddCircle>("/drone1/autopilot/add_circle");
    add_lemniscate_client_ = this->create_client<pegasus_msgs::srv::AddLemniscate>("/drone1/autopilot/add_lemniscate");
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

        // Wait for the service to be available
        while (!arm_disarm_client_->wait_for_service(std::chrono::seconds(1))) {
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

        // Wait for the service to be available
        while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
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

        // Wait for the service to be available
        while (!position_hold_client_->wait_for_service(std::chrono::seconds(1))) {
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

void ConsoleNode::on_offboard_click() {

    std::thread([this]() {

        // Create and fill the request to switch to offboard mode
        auto request = std::make_shared<pegasus_msgs::srv::Offboard::Request>();

        // Wait for the service to be available
        while (!offboard_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        offboard_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Offboard response: %s", response->success ? "true" : "false");
        });

    }).detach();
}

void ConsoleNode::on_kill_switch_click() {

    std::thread([this]() {

        // Create and fill the request to kill the vehicle
        auto request = std::make_shared<pegasus_msgs::srv::KillSwitch::Request>();
        request->kill = true;

        // Wait for the service to be available
        while (!kill_switch_client_->wait_for_service(std::chrono::seconds(1))) {
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

void ConsoleNode::on_set_autopilot_mode(const std::string & mode) {

    std::thread([this, mode]() {

        // Create and fill the request to set the autopilot mode
        auto request = std::make_shared<pegasus_msgs::srv::SetMode::Request>();
        request->mode = mode;

        // Wait for the service to be available
        while(!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            if(!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        set_mode_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::SetMode>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Set mode response: %s", response->success ? "true" : "false");
        });
    }).detach();
}

void ConsoleNode::on_thrust_curve_click() {

    // Get the thrust values from the UI
    float throtle = console_ui_->get_throtle();

    // Set the control message
    thrust_curve_msg_.thrust = throtle;
    thrust_curve_msg_.attitude[0] = 0.0;
    thrust_curve_msg_.attitude[1] = 0.0;
    thrust_curve_msg_.attitude[2] = 0.0;

    // Start publishing on the setpoint thread if it is not already running
    if (!thrust_curve_mode_) {

        // Set the flag to true and start the thread
        thrust_curve_mode_ = true;
        thrust_curve_thread_ = std::thread([this] () {
            while(this->thrust_curve_mode_) {
                // Publish the setpoint reference message at 30 Hz
                this->attitude_rate_pub_->publish(thrust_curve_msg_);

                // Sleep for 33 ms
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        });

        // Check if we are already in offboard mode - if not, set the offboard mode
        if (console_ui_->status_.flight_mode != 7) {
            std::thread([this]() {

                // Create and fill the request to set the vehicle to offboard mode
                auto request = std::make_shared<pegasus_msgs::srv::Offboard::Request>();

                using namespace std::chrono_literals;

                // Wait for the service to be available
                while (!offboard_client_->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                // Send the request to the service
                offboard_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedFuture future) {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Set flight mode response: %s", response->success ? "true" : "false");
                });
            }).detach();
        }
    }
}

void ConsoleNode::on_thrust_curve_stop() {

    // If thrust curve mode was not enabled, then just return
    if(!thrust_curve_mode_) return;

    // Set the last attitude_rate message to 0 thrust
    thrust_curve_msg_.thrust = 0.0;

    // Sleep for a few seconds before killing the thrust curve thread, to ensure the vehicle has stopped
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Set the flag to false
    thrust_curve_mode_ = false;

    // Stop publishing the thrust commands in a separate thread and join it
    if(thrust_curve_thread_.joinable()) thrust_curve_thread_.join();

    // Kill the vehicle
    on_kill_switch_click();
}

bool ConsoleNode::is_thrust_curve_running() {
    return this->thrust_curve_mode_;
}

void ConsoleNode::on_setpoint_click() {

    // Get the setpoint values from the UI
    std::pair<Eigen::Vector3d, float> pos_yaw = console_ui_->get_setpoint();

    // Update the Setpoint Control message witht he appropriate reference
    setpoint_msg_.position[0] = pos_yaw.first[0];
    setpoint_msg_.position[1] = pos_yaw.first[1];
    setpoint_msg_.position[2] = pos_yaw.first[2];
    setpoint_msg_.yaw = pos_yaw.second;

    // Start publishing on the setpoint thread if it is not already running
    if (!setpoint_mode_) {

        // Set the flag to true and start the thread
        setpoint_mode_ = true;
        setpoint_thread_ = std::thread([this] () {
            while(this->setpoint_mode_) {
                // Publish the setpoint reference message at 30 Hz
                this->position_pub_->publish(setpoint_msg_);

                // Sleep for 33 ms
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        });

        // Check if we are already in offboard mode - if not, set the offboard mode
        if (console_ui_->status_.flight_mode != 7) {
            std::thread([this]() {

                // Create and fill the request to set the vehicle to offboard mode
                auto request = std::make_shared<pegasus_msgs::srv::Offboard::Request>();

                using namespace std::chrono_literals;

                // Wait for the service to be available
                while (!offboard_client_->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }

                // Send the request to the service
                offboard_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Offboard>::SharedFuture future) {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Set flight mode response: %s", response->success ? "true" : "false");
                });
            }).detach();
        }
    }
}

// Add trajectory segments to the autopilot to follow
void ConsoleNode::on_add_waypoint_click() {
    // TODO
}

void ConsoleNode::on_add_arc_click() {
    // TODO
}

void ConsoleNode::on_add_line_click() {
    // TODO
}

void ConsoleNode::on_add_circle_click() {
    // TODO
}

void ConsoleNode::on_add_lemniscate_click() {
    // TODO
}

void ConsoleNode::on_setpoint_stop() {

    // If setpoint mode was not enabled, then just return
    if(!setpoint_mode_) return;

    // Set the vehicle to Hold mode
    on_hold_click();

    // Set the setpoint variable to false, such that the setpoint thread will stop
    // and the vehicle enters the Hold mode
    setpoint_mode_ = false;

    // Stop publishing the setpoing in a separate thread and join it
    if(setpoint_thread_.joinable()) setpoint_thread_.join();
}

bool ConsoleNode::is_setpoint_running() {
    return this->setpoint_mode_;
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