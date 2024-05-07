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

#include <map>
#include <array>
#include <string>
#include <Eigen/Dense>

const std::map<uint8_t, std::string> fmu_status_map = {
    {0, "DISARMED"},
    {1, "ARMED"},
    {2, "KILL SWITCH ENGAGED"},
    {3, "POSITION HOLD ACTIVE"},
    {4, "LANDING"},
    {5, "LANDED"},
    {6, "ERROR"}
};

const std::map<uint8_t, std::string> fmu_landed_state_map = {
    {0, "UNKNOWN"},
    {1, "ON GROUND"},
    {2, "IN AIR"},
    {3, "TAKING OFF"},
    {4, "LANDING"}
};

const std::map<uint8_t, std::string> fmu_flight_mode_map = {
    {0, "UNKNOWN"},
    {1, "READY"},
    {2, "TAKEOFF"},
    {3, "HOLD"},
    {4, "MISSION"},
    {5, "RETURN TO LAUNCH"},
    {6, "LAND"},
    {7, "OFFBOARD"},
    {8, "FOLLOW_ME"},
    {9, "MANUAL"},
    {10, "ATL_CTL"},
    {11, "POS_CTL"},
    {12, "ACRO"},
    {13, "STABILIZED"},
    {14, "RATTITUDE"}
};

struct State {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d attitude_euler{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_inertial{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct Battery {
    float temperature{0.0};
    float voltage{0.0};
    float current{0.0};
    float percentage{0.0};
    float amps_hour_consumed{0.0};
};

struct Health {
    bool is_armable{false};
    bool acc_calibrated{false};
    bool mag_calibrated{false};
    bool local_position_ok{false};
    bool global_position_ok{false};
    bool home_position_ok{false};
};

struct RcStatus {
    bool available{false};
    float signal_strength{0.0};
};

struct FmuStatus {
    uint8_t system_id{0};
    bool armed{false};
    uint8_t landed_state{0};
    uint8_t flight_mode{0};
    Battery battery;
    Health health;
    RcStatus rc_status;
};

struct PositionControlWidgetData {
    std::array<std::string, 4> inputs{"", "", "", ""};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    float yaw{0.0};
};

struct ThrotleWidgetData {
    std::string input{""};
    float throtle{0.0};
};

struct AutopilotwidgetData {

    // ----------------------------------
    // Waypoint UI data
    // ----------------------------------
    std::array<std::string, 3> waypoint_pos_input{"", "", ""};
    std::string waypoint_yaw_input{""};

    Eigen::Vector3d waypoint{Eigen::Vector3d::Zero()};
    float waypoint_yaw{0.0};

    // ----------------------------------
    // Arc UI data
    // ----------------------------------
    std::array<std::string, 5> arc_pos_input{"", "", "", "", ""};
    std::string arc_speed_input{""};

    Eigen::Matrix<double, 5, 1> arc{Eigen::Matrix<double, 5, 1>::Zero()};
    float arc_speed{0.0};

    // ----------------------------------
    // Line UI data
    // ----------------------------------
    std::array<std::string, 6> line_pos_input{"", "", "", "", "", ""};
    std::string line_speed_input{""};

    Eigen::Matrix<double, 6, 1> line{Eigen::Matrix<double, 6, 1>::Zero()};
    float line_speed{0.0};

    // ----------------------------------
    // Circle UI data
    // ----------------------------------
    std::array<std::string, 4> circle_pos_input{"", "", "", ""};
    std::string circle_speed_input{""};

    Eigen::Matrix<double, 4, 1> circle{Eigen::Matrix<double, 4, 1>::Zero()};
    float circle_speed{0.0};

    // ----------------------------------
    // Lemniscate UI data
    // ----------------------------------
    std::array<std::string, 4> lemniscate_pos_input{"", "", "", ""};
    std::string lemniscate_speed_input{""};

    Eigen::Matrix<double, 4, 1> lemniscate{Eigen::Matrix<double, 4, 1>::Zero()};
    float lemniscate_speed{0.0};
};