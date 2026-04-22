/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
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

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/waypoint.hpp"

namespace autopilot {

class SmoothWaypointMode : public autopilot::Mode {

public:

    ~SmoothWaypointMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The waypoint service callback
    void waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response);

    // Auxiliary function to compute the desired position, velocity, acceleration and jerk at a given gamma value
    Eigen::Vector3d desired_position(double gamma) const;
    Eigen::Vector3d desired_velocity(double gamma) const;
    Eigen::Vector3d desired_acceleration(double gamma) const;
    Eigen::Vector3d desired_jerk(double gamma) const;

    double gamma_dot(double gamma) const;
    double gamma_ddot(double gamma) const;
    double gamma_dddot(double gamma) const;

    // Check if the waypoint is already set
    bool waypoint_set_{false};

    // The target position and attitude waypoint to be at
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    double target_yaw_{0.0f};
    double target_speed_{0.5f}; // The target max speed in m/s

    // The start position and attitude to use as the start of the trajectory
    Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
    double start_yaw_{0.0f};

    // The slope of the line between the initial position of the vehicle and the target position
    Eigen::Vector3d trajectory_slope_{Eigen::Vector3d::Zero()};

    // Virtual target that integrates from zero to 1 over the trajectory duration
    double gamma_{0.0f};

    // Speed profile parameters
    double gamma_dot_max_{0.0f};
    double k_{0.3f};  

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::Waypoint>::SharedPtr waypoint_service_{nullptr};
};

} // namespace autopilot