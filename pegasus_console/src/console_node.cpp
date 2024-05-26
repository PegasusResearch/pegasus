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
#include "console_node.hpp"
#include "pegasus_utils/rotations.hpp"

ConsoleNode::ConsoleNode(const std::string vehicle_namespace, const unsigned int vehicle_id=1) : rclcpp::Node("pegasus_console") {
    
    // Initialize the vehicle namespace
    vehicle_namespace_ = std::string(vehicle_namespace + std::to_string(vehicle_id)); 

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
    config_.on_reset_path_click = std::bind(&ConsoleNode::on_reset_path_click, this);

    // Initialize the console UI
    console_ui_ = std::make_unique<ConsoleUI>(config_);
}

ConsoleNode::~ConsoleNode() {}

void ConsoleNode::initialize_subscribers() {

    this->declare_parameter<std::string>("console.subscribers.onboard.status", vehicle_namespace_ + std::string("/fmu/status"));
    this->declare_parameter<std::string>("console.subscribers.onboard.state", vehicle_namespace_ + std::string("/fmu/filter/state"));
    this->declare_parameter<std::string>("console.subscribers.autopilot.status", vehicle_namespace_ + std::string("/autopilot/status"));

    // Status of the vehicle
    status_sub_ = this->create_subscription<pegasus_msgs::msg::Status>(
        this->get_parameter("console.subscribers.onboard.status").as_string(), 
        rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::status_callback, this, std::placeholders::_1));
    
    // Subscribe to the state of the vehicle given by its internal EKF filter
    filter_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("console.subscribers.onboard.state").as_string(), 
        rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::state_callback, this, std::placeholders::_1));

    // Status of the autopilot
    autopilot_status_sub_ = this->create_subscription<pegasus_msgs::msg::AutopilotStatus>(
        this->get_parameter("console.subscribers.autopilot.status").as_string(), 
        rclcpp::SensorDataQoS(), std::bind(&ConsoleNode::autopilot_status_callback, this, std::placeholders::_1));
}

void ConsoleNode::initialize_publishers() {

    this->declare_parameter<std::string>("console.publishers.onboard.attitude_rate", vehicle_namespace_ + std::string("/fmu/in/throtle/attitude_rate"));
    this->declare_parameter<std::string>("console.publishers.onboard.position", vehicle_namespace_ + std::string("/fmu/in/position"));

    // Publish the attitude setpoints to the vehicle for thrust curve control
    attitude_rate_pub_ = this->create_publisher<pegasus_msgs::msg::ControlAttitude>(
        this->get_parameter("console.publishers.onboard.attitude_rate").as_string(), 
        rclcpp::SensorDataQoS());

    // Publish the setpoints to the vehicle
    position_pub_ = this->create_publisher<pegasus_msgs::msg::ControlPosition>(
        this->get_parameter("console.publishers.onboard.position").as_string(), 
        rclcpp::SensorDataQoS());
}

void ConsoleNode::initialize_services() {

    this->declare_parameter<std::string>("console.services.onboard.arm_disarm", vehicle_namespace_ + std::string("/fmu/arm"));
    this->declare_parameter<std::string>("console.services.onboard.land", vehicle_namespace_ + std::string("/fmu/land"));
    this->declare_parameter<std::string>("console.services.onboard.kill_switch", vehicle_namespace_ + std::string("/fmu/kill_switch"));
    this->declare_parameter<std::string>("console.services.onboard.position_hold", vehicle_namespace_ + std::string("/fmu/hold"));
    this->declare_parameter<std::string>("console.services.onboard.offboard", vehicle_namespace_ + std::string("/fmu/offboard"));

    this->declare_parameter<std::string>("console.services.autopilot.set_mode", vehicle_namespace_ + std::string("/autopilot/change_mode"));
    this->declare_parameter<std::string>("console.services.autopilot.set_waypoint", vehicle_namespace_ + std::string("/autopilot/set_waypoint"));
    this->declare_parameter<std::string>("console.services.autopilot.add_arc", vehicle_namespace_ + std::string("/autopilot/trajectory/add_arc"));
    this->declare_parameter<std::string>("console.services.autopilot.add_line", vehicle_namespace_ + std::string("/autopilot/trajectory/add_line"));
    this->declare_parameter<std::string>("console.services.autopilot.add_circle", vehicle_namespace_ + std::string("/autopilot/trajectory/add_circle"));
    this->declare_parameter<std::string>("console.services.autopilot.add_lemniscate", vehicle_namespace_ + std::string("/autopilot/trajectory/add_lemniscate"));
    this->declare_parameter<std::string>("console.services.autopilot.reset_path", vehicle_namespace_ + std::string("/autopilot/trajectory/reset"));

    // Create the service clients
    arm_disarm_client_ = this->create_client<pegasus_msgs::srv::Arm>(this->get_parameter("console.services.onboard.arm_disarm").as_string());
    land_client_ = this->create_client<pegasus_msgs::srv::Land>(this->get_parameter("console.services.onboard.land").as_string());
    kill_switch_client_ = this->create_client<pegasus_msgs::srv::KillSwitch>(this->get_parameter("console.services.onboard.kill_switch").as_string());
    position_hold_client_ = this->create_client<pegasus_msgs::srv::PositionHold>(this->get_parameter("console.services.onboard.position_hold").as_string());
    offboard_client_ = this->create_client<pegasus_msgs::srv::Offboard>(this->get_parameter("console.services.onboard.offboard").as_string());

    // Create the service clients for the autopilot
    set_mode_client_ = this->create_client<pegasus_msgs::srv::SetMode>(this->get_parameter("console.services.autopilot.set_mode").as_string());

    waypoint_client_ = this->create_client<pegasus_msgs::srv::Waypoint>(this->get_parameter("console.services.autopilot.set_waypoint").as_string());
    add_arc_client_ = this->create_client<pegasus_msgs::srv::AddArc>(this->get_parameter("console.services.autopilot.add_arc").as_string());
    add_line_client_ = this->create_client<pegasus_msgs::srv::AddLine>(this->get_parameter("console.services.autopilot.add_line").as_string());
    add_circle_client_ = this->create_client<pegasus_msgs::srv::AddCircle>(this->get_parameter("console.services.autopilot.add_circle").as_string());
    add_lemniscate_client_ = this->create_client<pegasus_msgs::srv::AddLemniscate>(this->get_parameter("console.services.autopilot.add_lemniscate").as_string());
    reset_path_client_ = this->create_client<pegasus_msgs::srv::ResetPath>(this->get_parameter("console.services.autopilot.reset_path").as_string());
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
    
    // Get the current waypoint from the UI
    auto autopilot_data = console_ui_->get_autopilot_data();
    Eigen::Vector3d waypoint = autopilot_data.waypoint;
    float yaw = autopilot_data.waypoint_yaw; 

    // Create and fill the request to add a waypoint to the autopilot
    std::thread([this, waypoint, yaw]() {
        
        auto request = std::make_shared<pegasus_msgs::srv::Waypoint::Request>();
        request->position[0] = waypoint[0];
        request->position[1] = waypoint[1];
        request->position[2] = waypoint[2];
        request->yaw = yaw;

        // Wait for the service to be available
        while (!waypoint_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        waypoint_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::Waypoint>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Waypoint response: %s", response->success ? "true" : "false");
        });
    }).detach();

}

void ConsoleNode::on_add_arc_click() {
    
    // Get the current arc configuration from the UI
    auto autopilot_data = console_ui_->get_autopilot_data();
    Eigen::Matrix<double, 5, 1> arc = autopilot_data.arc;
    float arc_speed = autopilot_data.arc_speed; 

    // Create and fill the request to add an arc to the autopilot
    std::thread([this, arc, arc_speed]() {
        
        auto request = std::make_shared<pegasus_msgs::srv::AddArc::Request>();
        
        // Set the starting position in a 2D plane
        request->start[0] = arc[0];
        request->start[1] = arc[1];

        // Set the center of the arc in 3D space
        request->center[0] = arc[2];
        request->center[1] = arc[3];
        request->center[2] = arc[4];

        // Set the normal vector of the plane in which the arc lies
        request->normal[0] = 0.0;
        request->normal[1] = 0.0;
        request->normal[2] = 1.0;

        // Set the speed of the arc in m/s
        request->speed.type = "constant";
        request->speed.parameters = std::vector<double>();
        request->speed.parameters.push_back(arc_speed);

        // Set the direction of the arc
        request->clockwise_direction = true;

        // Wait for the service to be available
        while (!add_arc_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        add_arc_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::AddArc>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Add Arc section response: %s", response->success ? "true" : "false");
        });
    }).detach();
}

void ConsoleNode::on_add_line_click() {
    
    // Get the current line configuration from the UI
    auto autopilot_data = console_ui_->get_autopilot_data();
    Eigen::Matrix<double, 6, 1> line = autopilot_data.line;
    float line_speed = autopilot_data.line_speed; 

    // Create and fill the request to add a line to the autopilot
    std::thread([this, line, line_speed]() {
        
        auto request = std::make_shared<pegasus_msgs::srv::AddLine::Request>();
        
        // Set the starting position in a 2D plane
        request->start[0] = line[0];
        request->start[1] = line[1];
        request->start[2] = line[2];

        // Set the end of the line in 3D space
        request->end[0] = line[3];
        request->end[1] = line[4];
        request->end[2] = line[5];

        // Set the speed of the arc in m/s
        request->speed.type = "constant";
        request->speed.parameters = std::vector<double>();
        request->speed.parameters.push_back(line_speed);

        // Wait for the service to be available
        while (!add_line_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        add_line_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::AddLine>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Add Line section response: %s", response->success ? "true" : "false");
        });
    }).detach();

}

void ConsoleNode::on_add_circle_click() {
    
    // Get the current circle configuration from the UI
    auto autopilot_data = console_ui_->get_autopilot_data();
    Eigen::Matrix<double, 4, 1> circle = autopilot_data.circle;
    float circle_speed = autopilot_data.circle_speed; 

    // Create and fill the request to add a circle to the autopilot
    std::thread([this, circle, circle_speed]() {

        auto request = std::make_shared<pegasus_msgs::srv::AddCircle::Request>();
        
        // Set the starting position in a 2D plane
        request->center[0] = circle[0];
        request->center[1] = circle[1];
        request->center[2] = circle[2];
        request->radius = circle[3];

        // Set the normal of the circle in 3D space
        request->normal[0] = 0.0;
        request->normal[1] = 0.0;
        request->normal[2] = 1.0;

        // Set the speed of the circle in m/s
        request->speed.type = "constant";
        request->speed.parameters = std::vector<double>();
        request->speed.parameters.push_back(circle_speed);

        // Wait for the service to be available
        while (!add_circle_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        add_circle_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::AddCircle>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Add Circle section response: %s", response->success ? "true" : "false");
        });
    }).detach();
}

void ConsoleNode::on_add_lemniscate_click() {
    
    // Get the current lemniscate configuration from the UI
    auto autopilot_data = console_ui_->get_autopilot_data();
    Eigen::Matrix<double, 4, 1> lemniscate = autopilot_data.lemniscate;
    float lemniscate_speed = autopilot_data.lemniscate_speed; 

    // Create and fill the request to add a lemniscate to the autopilot
    std::thread([this, lemniscate, lemniscate_speed]() {

        auto request = std::make_shared<pegasus_msgs::srv::AddLemniscate::Request>();
        
        // Set the starting position in a 2D plane
        request->center[0] = lemniscate[0];
        request->center[1] = lemniscate[1];
        request->center[2] = lemniscate[2];
        request->radius = lemniscate[3];

        // Set the normal of the lemniscate in 3D space
        request->normal[0] = 0.0;
        request->normal[1] = 0.0;
        request->normal[2] = 1.0;

        // Set the speed of the lemniscate in m/s
        request->speed.type = "constant";
        request->speed.parameters = std::vector<double>();
        request->speed.parameters.push_back(lemniscate_speed);

        // Wait for the service to be available
        while (!add_lemniscate_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        add_lemniscate_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::AddLemniscate>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Add Lemniscate  section response: %s", response->success ? "true" : "false");
        });
    }).detach();
}

void ConsoleNode::on_reset_path_click() {

    // Create and fill the request to add a waypoint to the autopilot
    std::thread([this]() {
        
        auto request = std::make_shared<pegasus_msgs::srv::ResetPath::Request>();

        // Wait for the service to be available
        while (!reset_path_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Send the request to the service
        reset_path_client_->async_send_request(request, [this](rclcpp::Client<pegasus_msgs::srv::ResetPath>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Reset path response: %s", response->success ? "true" : "false");
        });
    }).detach();
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

void ConsoleNode::autopilot_status_callback(const pegasus_msgs::msg::AutopilotStatus::ConstSharedPtr msg) {
    // Update the current autopilot mode
    console_ui_->autopilot_mode_ = msg->mode;
}