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

/**
 * @brief Method that given a desired acceleration to apply to the multirotor,
 * its mass and desired yaw angle (in radian), computes the desired attitude to apply to the vehicle.
 * It returns an Eigen::Vector4d object which contains [roll, pitch, yaw, thrust]
 * with each element expressed in the following units [rad, rad, rad, Newton] respectively.
 * 
 * @param acceleration The desired acceleration to apply to the vehicle in m/s^2
 * @param mass The mass of the vehicle in Kg
 * @param yaw The desired yaw angle of the vehicle in radians
 * @return Eigen::Vector4d object which contains [roll, pitch, yaw, thrust]
 * with each element expressed in the following units [rad, rad, rad, Newton] respectively.
 */
inline Eigen::Vector4d get_attitude_thrust_from_acceleration(const Eigen::Vector3d & acceleration, double mass, double yaw) {

    Eigen::Matrix3d RzT;
    Eigen::Vector3d r3d, u_bar;
    Eigen::Vector4d attitude_thrust;

    /* Compute the normalized thrust and r3d vector */
    double T = mass * acceleration.norm();
    r3d = - acceleration / acceleration.norm();

    /* Compute the rotation matrix about the Z-axis */
    RzT << cos(yaw), sin(yaw), 0.0,
          -sin(yaw), cos(yaw), 0.0,
                0.0,      0.0, 1.0;

    /* Compute the normalized rotation */
    u_bar = RzT * r3d;

    /* Compute the actual attitude and setup the desired thrust to apply to the vehicle */
    attitude_thrust << asin(-u_bar[1]), atan2(u_bar[0], u_bar[2]), yaw, T;
    return attitude_thrust;
}