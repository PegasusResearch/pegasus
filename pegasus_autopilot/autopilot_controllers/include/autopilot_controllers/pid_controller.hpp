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
#pragma once

#include <Eigen/Core>

// Library that implements PID controllers
#include "autopilot_controllers/pid.hpp"

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// ROS2 messages
#include "pegasus_msgs/msg/pid_statistics.hpp"
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"

#include <autopilot/controller.hpp>

namespace autopilot {

class PIDController : public autopilot::Controller {

public:

    ~PIDController();

    void initialize() override;
    void reset_controller() override;
    void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate=0, double dt=0) override;
    void set_attitude(const Eigen::Vector3d& attitude, double thrust_force, double dt=0) override;
    void set_attitude_rate(const Eigen::Vector3d& attitude_rate, double thrust_force, double dt=0) override;
    
protected:

    // Update the statistics of the PID controllers
    void update_statistics(const Eigen::Vector3d & position_ref);

    // The mass of the vehicle
    double mass_;

    // The PID controllers for position tracking
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_{nullptr};

    // ROS2 messages
    pegasus_msgs::msg::ControlAttitude attitude_msg_;
    pegasus_msgs::msg::ControlAttitude attitude_rate_msg_;
    pegasus_msgs::msg::PidStatistics pid_statistics_msg_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::PidStatistics>::SharedPtr statistics_pub_{nullptr};
};

}