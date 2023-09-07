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
#include "section.hpp"

namespace Pegasus::Paths {

class Arc : public Section {

public:

    using SharedPtr = std::shared_ptr<Arc>;
    using UniquePtr = std::unique_ptr<Arc>;
    using WeakPtr = std::weak_ptr<Arc>;

    /**
     * @brief Construct a new Arc section object
     * @param vehicle_speed The vehicle speed object
     * @param start The starting point where the arc is defined
     * @param center The center point of the arc
     * @param The normal vector that defines the plane where the 2D arc will be placed
     * @param clockwise_direction Whether the arc should be performed in clock or anti-clockwise direction (default=true)
     */
    Arc(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector2d & start, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const bool clockwise_direction=true);

    /**
     * @brief Construct a new Arc section object
     * @param vehicle_speed The vehicle speed object
     * @param start The starting point where the arc is defined
     * @param center The center point of the arc
     * @param clockwise_direction Whether the arc should be performed in clock or anti-clockwise direction (default=false)
     */
    Arc(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector2d & start, const Eigen::Vector3d & center, const bool clockwise_direction=true);

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
     */
    virtual Eigen::Vector3d pd(double gamma) override;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
     */
    virtual Eigen::Vector3d d_pd(double gamma) override;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
     */
    virtual Eigen::Vector3d dd_pd(double gamma) override;

    /** 
     * @brief Override and just returns 0.0
     * @param gamma The path parameter
     * @return A double with the line curvature  = 0
     */
    virtual double curvature(double gamma) override;

private:

    /**
     * @brief The starting point of the arc
     */
    Eigen::Vector2d start_;

    /**
     * @brief The center point of the arc
     */
    Eigen::Vector3d center_;

    /**
     * @brief The direction of the arc - clockwise our anti-clockwise
     */
    double clockwise_direction_{1.0};

    /**
     * @brief The radius of the arc
     */
    double radius_;

    /**
     * @brief The curvature of the arc
     */
    double curvature_;

    /**
     * @brief The angle of the circle corresponding to the initial position
     */
    double init_angle_;

    /**
     * @brief The normal vector of the plane where the circle will be located. By default
     * the circle will be in the xy-plane located in z=center[2], without any fancy rotation
     * applied to it. If the normal vector has some other value, then a rotation will be applied
     * to the plane
     */
    Eigen::Vector3d normal_{0.0, 0.0, 1.0};

    /**
     * @brief The rotation matrix to apply based on the normal vector to the plane where the circle
     * should be inscribed. By default, this is the identity matrix
     */
    Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};

};
}