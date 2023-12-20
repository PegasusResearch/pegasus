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
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
     */ 
    virtual Eigen::Vector3d pd(const double gamma) const = 0;
    virtual Eigen::Vector3d d_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }
    virtual Eigen::Vector3d d2_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }
    virtual Eigen::Vector3d d3_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }
    virtual Eigen::Vector3d d4_pd(const double gamma) const { return Eigen::Vector3d::Zero(); }

    /**
     * @brief Default method for getting the desired vehicle speed for a particular 
     * location in the path section (in m/s)
     * @param gamma The path parameter
     * @return double The desired vehicle speed (in m/s)
     */
    virtual double vehicle_speed(double gamma) const = 0;

    virtual double vd(const double gamma) const = 0;
    virtual double d_vd(const double gamma) const { return 0.0; };
    virtual double d2_vd(const double gamma) const { return 0.0; };

protected:

    /**
     * @brief The minimum and maximum value of the variable that parameterizes the trajectory
     */
    double min_gamma_{0.0};
    double max_gamma_{1.0};

};

} // namespace autopilot