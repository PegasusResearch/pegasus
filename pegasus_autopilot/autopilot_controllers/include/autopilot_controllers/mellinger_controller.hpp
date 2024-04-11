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

#include <Eigen/Core>

// Library that implements PID controllers
#include "autopilot_controllers/pid.hpp"

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// ROS2 messages
#include "pegasus_msgs/msg/control_attitude.hpp"
#include "pegasus_msgs/msg/control_position.hpp"
#include "pegasus_msgs/msg/mellinger_statistics.hpp"

#include <autopilot/controller.hpp>

namespace autopilot {

/**
 * @brief A nonlinear controller class. It implements a nonlinear controller that allows a vehicle to track
 *  aggressive trajectories. This controlers is well described in the papers
 *   
 * [1] J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," 
 * 2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athens, Greece, 2021, 
 * pp. 137-145, doi: 10.1109/ICUAS51884.2021.9476757.
 * [2] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 
 * 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, 
 * pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
 * [3] T. Lee, M. Leok and N. H. McClamroch, "Geometric Tracking Control of a Quadrotor UAV on SE(3),"
 * 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010, 
 * pp. 5420-5425, doi: 10.1109/CDC.2010.5717652.
 */
class MellingerController : public autopilot::Controller {

public:

    ~MellingerController();

    void initialize() override;
    void reset_controller() override;
    void set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate=0, double dt=0) override;
    void set_attitude(const Eigen::Vector3d& attitude, double thrust_force, double dt=0) override;
    void set_attitude_rate(const Eigen::Vector3d& attitude_rate, double thrust_force, double dt=0) override;
    
protected:

    // Update the statistics of the controller
    void update_statistics(const Eigen::Vector3d & position_ref, const Eigen::Vector3d & rotation_error, const Eigen::Vector3d & desired_angular_rate, double thrust_reference, const Eigen::Vector3d & attitude_rate_reference);

    // The mass of the vehicle
    double mass_;

    // The PID controllers for position tracking
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_{nullptr};

    // Gains for the attitude controller
    Eigen::Matrix3d kr_;

    // ROS2 messages
    pegasus_msgs::msg::ControlAttitude attitude_msg_;
    pegasus_msgs::msg::ControlAttitude attitude_rate_msg_;
    pegasus_msgs::msg::MellingerStatistics statistics_msg_;

    // ROS2 publishers
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_publisher_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::MellingerStatistics>::SharedPtr statistics_pub_{nullptr};
};

}