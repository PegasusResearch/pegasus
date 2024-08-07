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

#include "state.hpp"

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Custom ROS2 messages and services
#include "pegasus_msgs/srv/reset_path.hpp"

namespace autopilot {

class TrajectoryManager {

public:

    using SharedPtr = std::shared_ptr<TrajectoryManager>;
    using UniquePtr = std::unique_ptr<TrajectoryManager>;
    using WeakPtr = std::weak_ptr<TrajectoryManager>;

    // Configuration for the trajectory manager
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle      
        std::function<VehicleStatus()> get_vehicle_status;                      // Function pointer to get the current status of the vehicle  
        std::function<VehicleConstants()> get_vehicle_constants;                // Function pointer to get the current dynamical constants of the vehicle    
    };

    
    inline void initialize_trajectory_manager(const TrajectoryManager::Config& config) {
        
        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        get_vehicle_status = config.get_vehicle_status;
        get_vehicle_constants = config.get_vehicle_constants;

        // Initialize the derived class
        initialize();
    }

    virtual void initialize() = 0;

    /**
     * @brief This function returns the desired position of the vehicle at a given time
     * provided the parameter gamma which paramaterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired position of the vehicle at a given time (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d pd(const double gamma) const { throw std::runtime_error("pd() not implemented in TrajectoryManager"); }

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
     * @brief This function returns the desired position of the vehicle at any given time
     * provides the parametric parameter gamma which paramterizes the trajectory
     * @param gamma The parameter that paramaterizes the trajectory
     * @return Eigen::Vector3d The desired position in NED of the vehicle in the inertial frame (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d position(const double gamma) const { 
        return pd(gamma); 
    }

    /**
     * @brief This function returns the desired velocity of the vehicle at any given time
     * provides the parametric parameter gamma which paramterizes the trajectory and its time derivative
     * @param gamma The parameter that paramaterizes the trajectory
     * @param d_gamma The first time derivative of the path parameter
     * @return Eigen::Vector3d The desired velocity in NED of the vehicle in the inertial frame (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d velocity(const double gamma, const double d_gamma) const {
        return d_pd(gamma) * d_gamma;
    }

    /**
     * @brief This function returns the desired acceleration of the vehicle at any given time
     * provides the parametric parameter gamma which paramterizes the trajectory and its time derivative
     * @param gamma The parameter that paramaterizes the trajectory
     * @param d_gamma The first time derivative of the path parameter
     * @param d2_gamma The second time derivative of the path parameter
     * @return Eigen::Vector3d The desired acceleration in NED of the vehicle in the inertial frame (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d acceleration(const double gamma, const double d_gamma, const double d2_gamma=0) const {
        return (d2_pd(gamma) * std::pow(d_gamma, 2)) + (d_pd(gamma) * std::pow(d2_gamma, 2));
    }

    /**
     * @brief This function returns the desired jerk of the vehicle at any given time
     * provides the parametric parameter gamma which paramterizes the trajectory and its time derivative
     * @param gamma The parameter that paramaterizes the trajectory
     * @param d_gamma The first time derivative of the path parameter
     * @param d2_gamma The second time derivative of the path parameter
     * @param d3_gamma The third time derivative of the path parameter
     * @return Eigen::Vector3d The desired jerk in NED of the vehicle in the inertial frame (Eigen::Vector3d)
     */
    virtual Eigen::Vector3d jerk(const double gamma, const double d_gamma, const double d2_gamma=0, const double d3_gamma=0) const {
        return (d3_pd(gamma) * std::pow(d_gamma, 3)) + (3 * d2_pd(gamma) * d_gamma * d2_gamma) + (d_pd(gamma) * d3_gamma);
    }

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
    virtual double vehicle_speed(const double gamma) const {
        throw std::runtime_error("vehicle_speed() not implemented in TrajectoryManager");
    }
    
    /**
     * @brief This function returns the desired vehicle speed in the trajectory frame
     * (Note that this is not expressed in m/s as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired speed of the vehicle at a given time (double)
     */
    virtual double vd(const double gamma) const {
        throw std::runtime_error("vd() not implemented in TrajectoryManager");
    }

    /**
     * @brief This function returns the desired vehicle acceleration in the trajectory frame
     * (Note that this is not expressed in m/s^2 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired acceleration of the vehicle at a given time (double)
     */
    virtual double d_vd(const double gamma) const { return 0.0; }

    /**
     * @brief This function returns the desired vehicle jerk in the trajectory frame
     * (Note that this is not expressed in m/s^3 as the trajectory can be normalized between 0-1)
     * @param gamma The parameter that paramaterizes the trajectory
     * @return The desired jerk of the vehicle at a given time (double)
     */
    virtual double d2_vd(const double gamma) const { return 0.0; }

    /**
     * @brief This function returns the minimum value of the trajectory parameter gamma
     * @return The minimum value of the trajectory parameter gamma (double)
     */
    virtual double min_gamma() const {
        throw std::runtime_error("min_gamma() not implemented in TrajectoryManager");
    }

    /**
     * @brief This function returns the maximum value of the trajectory parameter gamma
     * @return The maximum value of the trajectory parameter gamma (double)
     */
    virtual double max_gamma() const {
        throw std::runtime_error("max_gamma() not implemented in TrajectoryManager");
    }

    /**
     * @brief This functions returns whether the trajectory is empty or not
     * @return True if the trajectory is empty, false otherwise
     */
    virtual bool empty() const {
        throw std::runtime_error("is_empty() not implemented in TrajectoryManager");
    }

protected:

    // The ROS2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Function pointer to get the current state of the vehicle
    std::function<State()> get_vehicle_state{nullptr};
    std::function<VehicleStatus()> get_vehicle_status{nullptr};
    std::function<VehicleConstants()> get_vehicle_constants{nullptr};
};

} // namespace autopilot