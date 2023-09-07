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

#include <Eigen/Dense>

namespace autopilot {

// State of the vehicle
struct State {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};           // Position of the vehicle (FRD) in the world frame NED
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};           // Velocity of the vehicle (FRD) with respect to the world frame NED expressed in the world frame NED
    Eigen::Quaterniond attitude{1.0, 0.0, 0.0, 0.0};             // Attitude of the vehicle (FRD) with respect to the world frame NED expressed in the world frame NED
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};   // Angular velocity of the vehicle (FRD) with respect to the world frame NED expressed in the body frame FRD
};

// Status of the vehicle
struct VehicleStatus {
    bool armed{false};      // Whether the vehicle is armed
    bool flying{false};     // Whether the vehicle is flying
    bool offboard{false};   // Whether the vehicle is in offboard mode
};

// Dynamical constants of the vehicle
struct VehicleConstants {
    double mass{0.0};                                                         // Mass of the vehicle (in Kg)
    std::string thrust_curve_id{"None"};                                      // Thrust curve of the vehicle
    std::vector<std::string> thurst_curve_params{std::vector<std::string>()}; // Thrust curve parameters
    std::vector<double> thrust_curve_values{std::vector<double>()};           // Thrust curve values associated with the parameters
};

}


