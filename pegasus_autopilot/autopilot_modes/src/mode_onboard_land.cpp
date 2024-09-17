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
#include <thread>
#include "autopilot_modes/mode_onboard_land.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

OnboardLandMode::~OnboardLandMode() {}

void OnboardLandMode::initialize() {
    // Initialize the ROS 2 service clients
    node_->declare_parameter<std::string>("autopilot.OnboardLandMode.land_service", "land");

    /* ----- THIS CODE IS ONLY USED IN ROS FOXY --------*/
    std::string sub_node_name = "autopilot_land_client";
    auto options = rclcpp::NodeOptions()
        .start_parameter_services(false)
        .start_parameter_event_publisher(false)
        .arguments({"--ros-args", "-r", "__node:=" + sub_node_name, "--"});

    sub_node_ = rclcpp::Node::make_shared("_", options);

    land_client_ = sub_node_->create_client<pegasus_msgs::srv::Land>(node_->get_parameter("autopilot.OnboardLandMode.land_service").as_string(), rmw_qos_profile_system_default);
   
    // Log that the ArmMode has been initialized successfully 
    RCLCPP_INFO(this->node_->get_logger(), "OnboardLandMode initialized");
}

bool OnboardLandMode::enter() {

    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->target_pos[0] = curr_state.position[0];
    this->target_pos[1] = curr_state.position[1];
    this->target_pos[2] = curr_state.position[2];

    // Set the target yaw to the current yaw of the drone (in degrees)
    this->target_yaw_ = Pegasus::Rotations::rad_to_deg(Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude));

    // Send the Land request in a separate thread
    std::thread t(&OnboardLandMode::request_landing, this);
    t.detach();
    
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "OnboardLandMode: Land request sent");

    return true;
}

void OnboardLandMode::request_landing() {
    
    // Prepare the request to invoke the land service from the onboard microcontroller
    auto land_request = std::make_shared<pegasus_msgs::srv::Land::Request>();

    // Wait until the service is available
    while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the land service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->node_->get_logger(), "landing service not available, waiting again...");
    }

    // Send the land request asynchronously (with a callback binding to check the response)
    auto result = land_client_->async_send_request(land_request);

    if(rclcpp::spin_until_future_complete(sub_node_, result, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->node_->get_logger(), "Error while waiting for result from Land service request");
        return;
    }
    
    // Check if the vehicle was set to land mode successfully
    bool land_approved = result.get()->success == pegasus_msgs::srv::Land::Response::SUCCESS ? true : false;

    // Log the output of the service
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "Landing aproval: " << land_approved);
}


void OnboardLandMode::update(double dt) {

    // Ask the microcontroller to keep the position that we had when entering this mode
    // Set the controller to track the target position and attitude
    // If the land service is successfull, these controls are ignore. If not, this 
    // line may prevent the drone from falling mid-air
    this->controller_->set_position(this->target_pos, this->target_yaw_, dt);
}

bool OnboardLandMode::exit() {

    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::OnboardLandMode, autopilot::Mode)