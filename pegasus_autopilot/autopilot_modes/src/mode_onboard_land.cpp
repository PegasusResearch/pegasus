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
#include "autopilot_modes/mode_land.hpp"
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

    // Prepare the request to invoke the land service from the onboard microcontroller
    auto land_request = std::make_shared<pegasus_msgs::srv::Land::Request>();

    // Send the land request asynchronously
    auto result = land_client_->async_send_request(land_request);
    
    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool OnboardLandMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void OnboardLandMode::update(double) {
    // Do nothing while the internal onboard autopilot is landing the vehicle
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::OnboardLandMode, autopilot::Mode)