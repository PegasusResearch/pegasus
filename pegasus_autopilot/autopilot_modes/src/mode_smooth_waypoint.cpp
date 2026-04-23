/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2026, Marcelo Jacinto. All rights reserved.
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
#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_smooth_waypoint.hpp"

namespace autopilot {

SmoothTrajectory::SmoothTrajectory(double v_max, double a_max) {
    this->set_limits(v_max, a_max);
}

void SmoothTrajectory::set_limits(double v_max, double a_max) {
    this->v_max_ = std::max(0.0, v_max);
    this->a_max_ = std::max(1e-6, a_max);
}

double SmoothTrajectory::plan(const Eigen::Vector3d & start_pos, const Eigen::Vector3d & target_pos) {

    this->start_pos_ = start_pos;
    this->target_pos_ = target_pos;

    const Eigen::Vector3d delta = target_pos - start_pos;
    this->distance_total_ = delta.norm();

    // Safety check to avoid division by zero and invalid trajectories. 
    // If the distance is negligible or the max velocity is zero, we set 
    // all trajectory parameters to zero and return a duration of zero.
    if (this->distance_total_ < 1e-9 || this->v_max_ <= 0.0) {
        this->direction_.setZero();
        this->t_ramp_ = 0.0;
        this->t1_ = 0.0;
        this->t2_ = 0.0;
        this->tf_ = 0.0;
        this->v_peak_ = 0.0;
        this->dist_ramp_ = 0.0;
        return this->tf_;
    }

    // Compute the unit direction vector from start to target
    this->direction_ = delta / this->distance_total_;

    const double t_ramp_max = (M_PI * this->v_max_) / (2.0 * this->a_max_);
    const double dist_ramp_max = 0.5 * this->v_max_ * t_ramp_max;

    // Determine the peak velocity based on the total distance and the maximum acceleration. 
    // If the distance is large enough to allow reaching the maximum velocity, we use it. 
    // Otherwise, we compute a lower peak velocity that allows us to reach the target within 
    // the given distance while respecting the acceleration limits.
    if (this->distance_total_ >= 2.0 * dist_ramp_max) {
        this->v_peak_ = this->v_max_;
    } else {
        this->v_peak_ = std::sqrt((2.0 * this->a_max_ * this->distance_total_) / M_PI);
    }

    // Compute the time to ramp up to the peak velocity and the distance covered during the ramp phase.
    this->t_ramp_ = (M_PI * this->v_peak_) / (2.0 * this->a_max_);
    this->dist_ramp_ = 0.5 * this->v_peak_ * this->t_ramp_;

    if (this->distance_total_ >= 2.0 * this->dist_ramp_) {
        const double cruise_dist = this->distance_total_ - 2.0 * this->dist_ramp_;
        this->t1_ = this->t_ramp_;
        this->t2_ = this->t1_ + cruise_dist / this->v_peak_;
        this->tf_ = this->t2_ + this->t_ramp_;
    } else {
        this->t1_ = this->t_ramp_;
        this->t2_ = this->t1_;
        this->tf_ = 2.0 * this->t_ramp_;
    }

    return this->tf_;
}

double SmoothTrajectory::velocity(double t) const {
    const double tc = std::clamp(t, 0.0, this->tf_);

    // Safety check to avoid invalid trajectories.
    if (this->tf_ <= 0.0) {
        return 0.0;
    }

    // Check if we are in the acceleration phase
    if (tc < this->t1_) {
        return (this->v_peak_ / 2.0) * (1.0 - std::cos(M_PI * tc / this->t_ramp_));
    }

    // Check if we are in the constant velocity phase
    if (tc < this->t2_) {
        return this->v_peak_;
    }

    // Check if we are in the deceleration phase
    const double tau = tc - this->t2_;
    return (this->v_peak_ / 2.0) * (1.0 + std::cos(M_PI * tau / this->t_ramp_));
}

double SmoothTrajectory::distance(double t) const {
    const double tc = std::clamp(t, 0.0, this->tf_);

    if (this->tf_ <= 0.0) {
        return 0.0;
    }

    const auto S = [this](double x) {
        return (this->v_peak_ / 2.0) * (x - (this->t_ramp_ / M_PI) * std::sin(M_PI * x / this->t_ramp_));
    };

    if (tc < this->t1_) {
        return S(tc);
    }

    if (tc < this->t2_) {
        return this->dist_ramp_ + this->v_peak_ * (tc - this->t1_);
    }

    const double tau = tc - this->t2_;
    return this->dist_ramp_ + this->v_peak_ * (this->t2_ - this->t1_) + (S(this->t_ramp_) - S(this->t_ramp_ - tau));
}

SmoothTrajectory::State SmoothTrajectory::get_state(double t) const {
    State state;

    if (this->distance_total_ < 1e-9 || this->tf_ <= 0.0) {
        state.position = this->start_pos_;
        return state;
    }

    const double tc = std::clamp(t, 0.0, this->tf_);
    const double v = this->velocity(tc);
    const double dist = this->distance(tc);
    const double gamma = dist / this->distance_total_;

    state.position = this->start_pos_ + gamma * (this->target_pos_ - this->start_pos_);
    state.velocity = v * this->direction_;

    double acc_scalar = 0.0;
    if (tc < this->t1_) {
        acc_scalar = (this->v_peak_ * M_PI / (2.0 * this->t_ramp_)) * std::sin(M_PI * tc / this->t_ramp_);
    } else if (tc < this->t2_) {
        acc_scalar = 0.0;
    } else {
        const double tau = tc - this->t2_;
        acc_scalar = -(this->v_peak_ * M_PI / (2.0 * this->t_ramp_)) * std::sin(M_PI * tau / this->t_ramp_);
    }

    state.acceleration = acc_scalar * this->direction_;
    return state;
}

double SmoothTrajectory::duration() const {
    return this->tf_;
}

SmoothYawTrajectory::SmoothYawTrajectory(double omega_max, double d_omega_max)
: smooth_trajectory_(omega_max, d_omega_max) {
}

void SmoothYawTrajectory::set_limits(double omega_max, double d_omega_max) {
    this->smooth_trajectory_.set_limits(omega_max, d_omega_max);
}

double SmoothYawTrajectory::plan(double yaw_start, double yaw_target) {
    this->yaw_start_ = Pegasus::Rotations::wrapTopi(yaw_start);

    // Plan the shortest angular displacement in [-pi, pi].
    const double yaw_delta = Pegasus::Rotations::wrapTopi(yaw_target - this->yaw_start_);
    this->yaw_sign_ = (std::abs(yaw_delta) > 1e-9) ? ((yaw_delta > 0.0) ? 1.0 : -1.0) : 1.0;

    const Eigen::Vector3d start_yaw_state(0.0, 0.0, 0.0);
    const Eigen::Vector3d target_yaw_state(std::abs(yaw_delta), 0.0, 0.0);

    this->tf_ = this->smooth_trajectory_.plan(start_yaw_state, target_yaw_state);
    return this->tf_;
}

SmoothYawTrajectory::State SmoothYawTrajectory::get_state(double t) const {
    const SmoothTrajectory::State state = this->smooth_trajectory_.get_state(t);

    State yaw_state;
    yaw_state.yaw = Pegasus::Rotations::wrapTopi(this->yaw_start_ + this->yaw_sign_ * state.position.x());
    yaw_state.yaw_rate = this->yaw_sign_ * state.velocity.x();
    yaw_state.yaw_acceleration = this->yaw_sign_ * state.acceleration.x();
    return yaw_state;
}

double SmoothYawTrajectory::duration() const {
    return this->tf_;
}

SmoothWaypointMode::~SmoothWaypointMode() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

void SmoothWaypointMode::initialize() {

    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.SmoothWaypointMode.set_waypoint_service", "set_smooth_waypoint"); 
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.max_speed", 1.5);
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.max_acceleration", 2.0);
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.max_yaw_rate", 1.5);
    node_->declare_parameter<double>("autopilot.SmoothWaypointMode.max_yaw_acceleration", 2.0);

    // Set the class members from parameters (avoid local shadowing).
    double max_speed = node_->get_parameter("autopilot.SmoothWaypointMode.max_speed").as_double();
    double max_acceleration = node_->get_parameter("autopilot.SmoothWaypointMode.max_acceleration").as_double();
    this->max_yaw_rate_ = node_->get_parameter("autopilot.SmoothWaypointMode.max_yaw_rate").as_double();
    this->max_yaw_acceleration_ = node_->get_parameter("autopilot.SmoothWaypointMode.max_yaw_acceleration").as_double();

    // Set the max speed and acceleration for the trajectory planner
    trajectory_ = SmoothTrajectory(max_speed, max_acceleration);
    yaw_trajectory_ = SmoothYawTrajectory(this->max_yaw_rate_, this->max_yaw_acceleration_);

    // Create the service server for setting the waypoint
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(node_->get_parameter("autopilot.SmoothWaypointMode.set_waypoint_service").as_string(), std::bind(&SmoothWaypointMode::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));    
    
    // Log the initialization of the mode
    RCLCPP_INFO(this->node_->get_logger(), "SmoothWaypointMode initialized with max speed %f m/s, max acceleration %f m/s^2, max yaw rate %f rad/s, max yaw acceleration %f rad/s^2 and service server '%s'", max_speed, max_acceleration, this->max_yaw_rate_, this->max_yaw_acceleration_, node_->get_parameter("autopilot.SmoothWaypointMode.set_waypoint_service").as_string().c_str());
}

bool SmoothWaypointMode::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->waypoint_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Waypoint not set - cannot enter SmoothWaypoint mode.");
        return false;
    }

    // Set the trajectory to be followed by the controller (to start at the current position)
    this->set_trajectory_parameters();
    
    // Reset the waypoint flag (to make sure we do not enter twice in this mode without setting a new waypoint)
    this->waypoint_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

void SmoothWaypointMode::set_trajectory_parameters() {

    // Get the current vehicle position
    this->start_pos_ = this->get_vehicle_state().position;
    this->start_yaw_ = Pegasus::Rotations::yaw_from_quaternion(this->get_vehicle_state().attitude);

    // Plan the trajectory from the current position to the target position and get the trajectory duration
    this->T_max_ = this->trajectory_.plan(this->start_pos_, this->target_pos_);
    this->T_yaw_max_ = this->yaw_trajectory_.plan(this->start_yaw_, this->target_yaw_);
    this->t_ = 0.0;

    RCLCPP_INFO(this->node_->get_logger(), "Trajectory planned with position duration %f seconds and yaw duration %f seconds", this->T_max_, this->T_yaw_max_);
}

bool SmoothWaypointMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void SmoothWaypointMode::update(double dt) {
    
    // Update the trajectory time and saturate the trajectory time to the maximum trajectory duration to avoid invalid states
    const double total_duration = std::max(this->T_max_, this->T_yaw_max_);
    this->t_ = std::min(this->t_ + std::max(0.0, dt), total_duration);

    // Get the desired state from the trajectory planner
    const SmoothTrajectory::State state = this->trajectory_.get_state(this->t_);
    const SmoothYawTrajectory::State yaw_state = this->yaw_trajectory_.get_state(this->t_);

    // Set the controller to track the target position and attitude
    this->controller_->set_position(state.position, state.velocity, state.acceleration, Pegasus::Rotations::rad_to_deg(yaw_state.yaw), Pegasus::Rotations::rad_to_deg(yaw_state.yaw_rate), dt);
}

void SmoothWaypointMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->target_pos_[0] = request->position[0];
    this->target_pos_[1] = request->position[1];
    this->target_pos_[2] = request->position[2];
    this->target_yaw_ = Pegasus::Rotations::deg_to_rad(request->yaw);

    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f) with yaw %f", this->target_pos_[0], this->target_pos_[1], this->target_pos_[2], request->yaw);

    // Set the trajectory to be followed by the controller (to start at the current position)
    this->set_trajectory_parameters();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::SmoothWaypointMode, autopilot::Mode)