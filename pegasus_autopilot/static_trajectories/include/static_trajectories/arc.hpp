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

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Service to setup a line trajectory
#include "pegasus_msgs/srv/add_arc.hpp"

// Base class import for defining a static trajectory and the corresponding factory
#include <static_trajectory_manager/static_trajectory.hpp>
#include <static_trajectory_manager/static_trajectory_factory.hpp>

namespace autopilot {

class Arc : public StaticTrajectory {

public:

    using SharedPtr = std::shared_ptr<Arc>;
    using UniquePtr = std::unique_ptr<Arc>;
    using WeakPtr = std::weak_ptr<Arc>;

    Arc(const Eigen::Vector2d & start, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, double vehicle_speed, const bool clockwise_direction=true);

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
     */
    virtual Eigen::Vector3d pd(const double gamma) const override;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
     */
    virtual Eigen::Vector3d d_pd(const double gamma) const override;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
     */
    virtual Eigen::Vector3d d2_pd(const double gamma) const override;

    /**
     * @brief This function returns the desired yaw angle (in radians) of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired yaw angle of the vehicle at a given time (radians)
     */
    virtual double yaw(const double gamma) const override;

    /**
     * @brief This function returns the desired yaw rate (in radians/s) of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired yaw rate of the vehicle at a given time (radians/s)
     */
    virtual double d_yaw(const double gamma) const override;

    /**
     * @brief Get the vehicle speed progression (in m/s)
     * @param gamma The path parametric value
     */
    double vehicle_speed(const double gamma) const override;

    /**
     * @brief Get the desired speed progression for the path parametric value (expressed in the path frame)
     * @param gamma The path parametric value
     */
    double vd(const double gamma) const override;

protected:

    /** @brief The desired vehicle speed in m/s */
    double vehicle_speed_;

    /** @brief The starting point of the arc */
    Eigen::Vector2d start_;

    /** @brief The center point of the arc */
    Eigen::Vector3d center_;

    /** @brief The direction of the arc - clockwise our anti-clockwise */
    double clockwise_direction_{1.0};

    /** @brief The radius of the arc */
    double radius_;

    /** @brief The angle of the circle corresponding to the initial position */
    double init_angle_;

    /** @brief The normal vector of the plane where the circle will be located. By default
     * the circle will be in the xy-plane located in z=center[2], without any fancy rotation
     * applied to it. If the normal vector has some other value, then a rotation will be applied
     * to the plane */
    Eigen::Vector3d normal_{0.0, 0.0, 1.0};

    /** @brief The rotation matrix to apply based on the normal vector to the plane where the circle
     * should be inscribed. By default, this is the identity matrix */
    Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
};

class ArcFactory : public StaticTrajectoryFactory {

public:

    virtual void initialize() override;

protected:

    // Service callback to setup an arc trajectory
    void arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, pegasus_msgs::srv::AddArc::Response::SharedPtr response);

    // Service to append an arc trajectory to the trajectory manager
    rclcpp::Service<pegasus_msgs::srv::AddArc>::SharedPtr add_arc_service_{nullptr};
};

} // namespace autopilot