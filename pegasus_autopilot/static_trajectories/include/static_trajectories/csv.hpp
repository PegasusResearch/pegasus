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

#include <vector>
#include <memory>
#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Service to setup a line trajectory
#include "pegasus_msgs/srv/add_csv.hpp"

// Base class import for defining a static trajectory and the corresponding factory
#include <static_trajectory_manager/static_trajectory.hpp>
#include <static_trajectory_manager/static_trajectory_factory.hpp>

namespace autopilot {

class CSVTrajectory: public StaticTrajectory {

public:

    using SharedPtr = std::shared_ptr<CSVTrajectory>;
    using UniquePtr = std::unique_ptr<CSVTrajectory>;
    using WeakPtr = std::weak_ptr<CSVTrajectory>;

    CSVTrajectory(const std::string & filename, const Eigen::Vector3d & offset, bool check_z_negative);

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
     * @brief This function returns the desired jerk of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d d3_pd(const double gamma) const override;

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

    void parse_csv(const std::string & filename);
    int get_closest_index(const double gamma) const;

    // The vectors that stores the data that represents the trajectory
    std::vector<double> time_;
    
    std::vector<Eigen::Vector3d> pos_;
    std::vector<Eigen::Vector3d> vel_;
    std::vector<Eigen::Vector3d> acc_;
    std::vector<Eigen::Vector3d> jerk_;

    std::vector<double> yaw_;
    std::vector<double> yaw_rate_;

    // Get the rate at which the trajectory is sampled
    double dt_;
};

class CSVFactory: public StaticTrajectoryFactory {

public:

    virtual void initialize() override;

protected:

    // Service callback to setup a csv trajectory
    void csv_callback(const pegasus_msgs::srv::AddCsv::Request::SharedPtr request, pegasus_msgs::srv::AddCsv::Response::SharedPtr response);

    // Service to append a csv trajectory to the trajectory manager
    rclcpp::Service<pegasus_msgs::srv::AddCsv>::SharedPtr add_csv_service_{nullptr};
};

} //namespace autopilot