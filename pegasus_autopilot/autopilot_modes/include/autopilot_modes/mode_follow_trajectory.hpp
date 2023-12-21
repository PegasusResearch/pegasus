/****************************************************************************
 *
 *   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Pegasus nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <autopilot/mode.hpp>

// Library that implements PID controllers
#include "pid/pid.hpp"

// Messages for controller debugging
#include "pegasus_msgs/msg/pid_statistics.hpp"

namespace autopilot {

class FollowTrajectoryMode : public autopilot::Mode {

public:

    ~FollowTrajectoryMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    // Get the desired position, velocity and acceleration from the path
    void update_reference(double dt);
    bool check_finished();
    void update_statistics();

    // Publisher for the PID statistics
    pegasus_msgs::msg::PidStatistics pid_statistics_msg_;
    rclcpp::Publisher<pegasus_msgs::msg::PidStatistics>::SharedPtr statistics_pub_{nullptr};

    // The mass of the vehicle
    double mass_;

    // Set the progression speed of the parametric variable
    double gamma_{0.0};
    double d_gamma_{0.0};
    double dd_gamma_{0.0};
    
    // Set the desired targets to follow
    Eigen::Vector3d desired_position_{0.0, 0.0, 0.0};
    Eigen::Vector3d desired_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d desired_acceleration_{0.0, 0.0, 0.0};
    double desired_yaw_{0.0};

    // The PID controller to follow the path
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_{nullptr};
};
}