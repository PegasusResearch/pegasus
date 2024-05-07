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

#include <memory>
#include <Eigen/Core>

namespace autopilot {

/**
 * @brief The Trajectory class is an abstract class that defines the interface
 * to create a trajectory section that can be followed by a trajectory or path following controller
*/
class StaticTrajectory {

public:

    using SharedPtr = std::shared_ptr<StaticTrajectory>;
    using UniquePtr = std::unique_ptr<StaticTrajectory>;
    using WeakPtr = std::weak_ptr<StaticTrajectory>;

    /**
     * @brief This function returns the desired position of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired position of the vehicle at a given time (Eigen::Vector3d)
     */ 
    virtual Eigen::Vector3d pd(const double gamma) const = 0;

    /**
     * @brief This function returns the desired velocity of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired velocity of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }

    /**
     * @brief This function returns the desired acceleration of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired acceleration of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d2_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }

    /**
     * @brief This function returns the desired jerk of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d3_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }

    /**
     * @brief This function returns the desired snap of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired snap of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d4_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }

    /**
     * @brief This function returns the desired yaw angle (in radians) of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired yaw angle of the vehicle at a given time (radians)
     */
    virtual double yaw(const double gamma) const { return 0.0; }

    /**
     * @brief This function returns the desired yaw rate (in radians/s) of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired yaw rate of the vehicle at a given time (radians/s)
     */
    virtual double d_yaw(const double gamma) const { return 0.0; }

    /**
     * @brief This function returns the vehicle speed in m/s at any given position in the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired speed of the vehicle at a given time (double)
     */
    virtual double vehicle_speed(double gamma) const = 0;

    /**
     * @brief This function returns the desired vehicle speed in the trajectory frame
     * (Note that this is not expressed in m/s as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired speed of the vehicle at a given time (double)
     */
    virtual double vd(const double gamma) const = 0;

    /**
     * @brief This function returns the desired vehicle acceleration in the trajectory frame
     * (Note that this is not expressed in m/s^2 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired acceleration of the vehicle at a given time (double)
     */
    virtual double d_vd(const double gamma) const { return 0.0; };

    /**
     * @brief This function returns the desired vehicle jerk in the trajectory frame
     * (Note that this is not expressed in m/s^3 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (double)
     */
    virtual double d2_vd(const double gamma) const { return 0.0; };

    /**
     * @brief Getter for the minimum value of the variable that parameterizes the trajectory
     */
    inline double min_gamma() const { return min_gamma_; }

    /**
     * @brief Getter for the maximum value of the variable that parameterizes the trajectory
     */
    inline double max_gamma() const { return max_gamma_; }

protected:

    /**
     * @brief Construct a new Trajectory object
     */
    StaticTrajectory(double min_gamma=0.0, double max_gamma=1.0) : 
        min_gamma_(min_gamma), max_gamma_(max_gamma) {}

    /**
     * @brief The minimum and maximum value of the variable that parameterizes the trajectory
     */
    double min_gamma_;
    double max_gamma_;

};

} // namespace autopilot