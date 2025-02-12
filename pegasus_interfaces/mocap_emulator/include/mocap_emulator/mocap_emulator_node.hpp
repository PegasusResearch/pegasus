/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2025, Marcelo Jacinto. All rights reserved.
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

// ROS Libraries
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class MocapEmulatorNode : public rclcpp::Node {

public:

    MocapEmulatorNode();
    ~MocapEmulatorNode() {}

private:

    // Subscriber callbacks to get the current state of the vehicle
    void state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscriber_;

    // ROS2 messages
    geometry_msgs::msg::PoseStamped pose_msg_;
};
