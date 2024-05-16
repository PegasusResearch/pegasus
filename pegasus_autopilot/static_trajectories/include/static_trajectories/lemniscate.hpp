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

// Service to setup a lemniscate trajectory
#include "pegasus_msgs/srv/add_lemniscate.hpp"

// Base class import for defining a static trajectory and the corresponding factory
#include <static_trajectory_manager/static_trajectory.hpp>
#include <static_trajectory_manager/static_trajectory_factory.hpp>

namespace autopilot {

class Lemniscate : public StaticTrajectory {

public:

    using SharedPtr = std::shared_ptr<Lemniscate>;
    using UniquePtr = std::unique_ptr<Lemniscate>;
    using WeakPtr = std::weak_ptr<Lemniscate>;

    /** 
     * @brief Constructor for a new Lemniscate path section
     * @param center A 3D vector with the starting point for the line
     * @param normal The normal vector that defines the plane where the 2D lemniscate will be placed
     * @param radius The radius of the circle in meters (m)
     * @param vehicle_speed A shared pointer to a desired vehicle speed */
    Lemniscate(const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const double radius, const double vehicle_speed);

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     */
    Eigen::Vector3d pd(const double gamma) const override;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     */
    Eigen::Vector3d d_pd(const double gamma) const override;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     */
    Eigen::Vector3d d2_pd(const double gamma) const override;

    double vehicle_speed(const double gamma) const override;
    double vd(const double gamma) const override;

protected:

    /** @brief The desired vehicle speed in m/s */
    double vehicle_speed_;

    /** @brief The starting point of the line */
    Eigen::Vector3d center_;

    /** @brief The normal vector of the plane where the circle will be located. By default
     * the circle will be in the xy-plane located in z=center[2], without any fancy rotation
     * applied to it. If the normal vector has some other value, then a rotation will be applied
     * to the plane */
    Eigen::Vector3d normal_{0.0, 0.0, 1.0};

    /** @brief The rotation matrix to apply based on the normal vector to the plane where the circle
     * should be inscribed. By default, this is the identity matrix */
    Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};

    /** @brief The radius of the circle */
    double radius_;
};

class LemniscateFactory : public StaticTrajectoryFactory {

public:

    virtual void initialize() override;

protected:

    // Service callback to setup a line trajectory
    void lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, const pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response);

    // Service to append a line trajectory to the trajectory manager
    rclcpp::Service<pegasus_msgs::srv::AddLemniscate>::SharedPtr add_lemniscate_service_{nullptr};
};

} // namespace autopilot