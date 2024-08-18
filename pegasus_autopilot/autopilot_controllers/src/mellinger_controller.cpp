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
#include "autopilot_controllers/mellinger_controller.hpp"

#include <pegasus_utils/rotations.hpp>

namespace autopilot {

MellingerController::~MellingerController() {}

void MellingerController::initialize() {

    // Load the controller gains from the parameter server
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.kp", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.kd", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.ki", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.kr", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.min_output", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("autopilot.MellingerController.gains.max_output", std::vector<double>());

    auto kp = node_->get_parameter("autopilot.MellingerController.gains.kp").as_double_array();
    auto kd = node_->get_parameter("autopilot.MellingerController.gains.kd").as_double_array();
    auto ki = node_->get_parameter("autopilot.MellingerController.gains.ki").as_double_array();
    auto kr = node_->get_parameter("autopilot.MellingerController.gains.kr").as_double_array();
    auto min_output = node_->get_parameter("autopilot.MellingerController.gains.min_output").as_double_array();
    auto max_output = node_->get_parameter("autopilot.MellingerController.gains.max_output").as_double_array();

    // Safety check on the gains (make sure they are there)
    if(kp.size() != 3 || kd.size() != 3 || ki.size() != 3 || kr.size() != 3 || min_output.size() != 3 || max_output.size() != 3) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not read MellingerController position controller gains correctly.");
        throw std::runtime_error("Gains vector was empty");
    }

    // Log the gains
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kp = [" << kp[0] << ", " << kp[1] << ", " << kp[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kd = [" << kd[0] << ", " << kd[1] << ", " << kd[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: ki = [" << ki[0] << ", " << ki[1] << ", " << ki[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController gains: kr = [" << kr[0] << ", " << kr[1] << ", " << kr[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController min_output = [" << min_output[0] << ", " << min_output[1] << ", " << min_output[2] << "]");
    RCLCPP_INFO_STREAM(node_->get_logger(), "MellingerController max_output = [" << max_output[0] << ", " << max_output[1] << ", " << max_output[2] << "]");

    // Initialize the attitude rate gains
    kr_ = Eigen::Matrix3d::Identity();
    for(unsigned int i=0; i < 3; i++) kr_(i, i) = kr[i];

    // Create the 3 PID controllers for x, y and z axis with unitary feedforward gain for the acceleration
    Eigen::Vector3d kff(1.0, 1.0, 1.0);
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    mass_ = vehicle_constansts.mass;

    // Initialize the ROS 2 subscribers to the control topics
    node_->declare_parameter<std::string>("autopilot.MellingerController.publishers.control_attitude", "control_attitude");
    node_->declare_parameter<std::string>("autopilot.MellingerController.publishers.control_attitude_rate", "control_attitude_rate");
    node_->declare_parameter<std::string>("autopilot.MellingerController.publishers.debug_topic", "statistics/mellinger");

    // Create the publishers
    attitude_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.MellingerController.publishers.control_attitude").as_string(), rclcpp::SensorDataQoS());
    attitude_rate_publisher_ = node_->create_publisher<pegasus_msgs::msg::ControlAttitude>(node_->get_parameter("autopilot.MellingerController.publishers.control_attitude_rate").as_string(), rclcpp::SensorDataQoS());
    statistics_pub_ = node_->create_publisher<pegasus_msgs::msg::MellingerStatistics>(node_->get_parameter("autopilot.MellingerController.publishers.debug_topic").as_string(), 1);

    // Log that the MellingerController was initialized
    RCLCPP_INFO(node_->get_logger(), "MellingerController initialized");
}

void MellingerController::set_position(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk, const Eigen::Vector3d& snap, double yaw, double yaw_rate, double dt) {

    // Ignore snap references
    (void) snap;

    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Get the current attitude in quaternion and generate a rotation matrix
    Eigen::Matrix3d R = state.attitude.toRotationMatrix();

    // Get the attitude but in euler angles (for debugging purposes only)
    Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0);

    // Get the current yaw and yaw-rate from degrees to radians
    double yaw_rad = Pegasus::Rotations::deg_to_rad(yaw);
    double yaw_rate_rad = Pegasus::Rotations::deg_to_rad(yaw_rate);
    
    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = position - state.position;
    Eigen::Vector3d vel_error = velocity - state.velocity;

    Eigen::Vector3d external_force = Eigen::Vector3d(0.0, 0.0, 0.0);
    external_force[0] = acceleration[0];
    external_force[1] = acceleration[1];
    external_force[2] = acceleration[2] - 9.81;

    // Compute the desired force output using a PID scheme
    Eigen::Vector3d F_des;
    for(unsigned int i=0; i < 3; i++) F_des[i] = mass_ * controllers_[i]->compute_output(pos_error[i], vel_error[i], external_force[i], dt);

    // Compute the desired body-frame axis Z_b (b3d)
    // Check [3-eq.12] for more details
    Eigen::Vector3d Z_b_des = -F_des / F_des.norm();

    // Compute Y_C
    Eigen::Vector3d Y_C = Eigen::Vector3d(-sin(yaw_rad), cos(yaw_rad), 0.0);

    // Compute X_B_des
    Eigen::Vector3d X_b_des = Y_C.cross(Z_b_des);
    X_b_des = X_b_des / X_b_des.norm();

    // Compute Y_B_des
    Eigen::Vector3d Y_b_des = Z_b_des.cross(X_b_des);
    Y_b_des = Y_b_des / Y_b_des.norm();

    // Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
    Eigen::Matrix3d R_des;
    R_des.col(0) = X_b_des;
    R_des.col(1) = Y_b_des;
    R_des.col(2) = Z_b_des;

    // Get the desired Euler-angles (for debugging purposes only)
    Eigen::Vector3d euler_angles_des = R_des.eulerAngles(2, 1, 0);

    // Compute the rotation error
    Eigen::Matrix3d R_error = (R_des.transpose() * R) - (R.transpose() * R_des);

    // Compute the vee map of the rotation error and project into the coordinates of the manifold
    Eigen::Vector3d e_R;
    e_R << -R_error(1,2), R_error(0, 2), -R_error(0,1);
    e_R = 0.5 * e_R;

    // Get the desired total thrust (in Newtons) in Z_B direction (u_1)
    Eigen::Vector3d Z_B = R.col(2);
    double T = -F_des.dot(Z_B);

    // Compute the desired angular velocity for the feed-forward terms
    Eigen::Vector3d w_des;
    w_des(0) =  mass_ / T * Y_b_des.dot(jerk);
    w_des(1) = -mass_ / T * X_b_des.dot(jerk);
    w_des(2) = yaw_rate_rad * Z_b_des[2];

    // Compute the target attitude rate
    Eigen::Vector3d attitude_rate = w_des - (kr_ * e_R);

    // Convert the output to rad/s
    attitude_rate = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_rate[0]), 
        Pegasus::Rotations::rad_to_deg(attitude_rate[1]), 
        Pegasus::Rotations::rad_to_deg(attitude_rate[2]));

    // Send the attitude rate and thrust to the attitude-rate controller
    set_attitude_rate(attitude_rate, T);

    // Update and publish the statistics
    update_statistics(position, e_R, w_des, T, attitude_rate, euler_angles, euler_angles_des);
    statistics_pub_->publish(statistics_msg_);
}

void MellingerController::update_statistics(const Eigen::Vector3d & position_ref, const Eigen::Vector3d & rotation_error, const Eigen::Vector3d & desired_angular_rate, double thrust_reference, const Eigen::Vector3d & attitude_rate_reference, const Eigen::Vector3d & euler_angles, const Eigen::Vector3d & euler_angles_desired) {
    
    for(unsigned int i = 0; i < 3; i++) {

        // For each control axis [x, y, z]
        // Get the statistics from the controller object
        Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

        statistics_msg_.pid_statistics[i].dt = stats.dt;
        statistics_msg_.pid_statistics[i].reference = position_ref[i];
        // Fill the feedback errors
        statistics_msg_.pid_statistics[i].error_p = stats.error_p;
        statistics_msg_.pid_statistics[i].error_d = stats.error_d;
        statistics_msg_.pid_statistics[i].integral = stats.integral;
        statistics_msg_.pid_statistics[i].ff_ref = stats.ff_ref;

        // Fill the errors scaled by the gains
        statistics_msg_.pid_statistics[i].p_term = stats.p_term;
        statistics_msg_.pid_statistics[i].d_term = stats.d_term;
        statistics_msg_.pid_statistics[i].i_term = stats.i_term;
        statistics_msg_.pid_statistics[i].ff_term = stats.ff_term;

        // Fill the outputs of the controller
        statistics_msg_.pid_statistics[i].anti_windup_discharge = stats.anti_windup_discharge;
        statistics_msg_.pid_statistics[i].output_pre_sat = stats.output_pre_sat;
        statistics_msg_.pid_statistics[i].output = stats.output;

        // For each rotation axis [x, y, z]
        // Fill in the nonlinear errors 
        statistics_msg_.rotation_error[i] = rotation_error[i];
        statistics_msg_.desired_angular_rate[i] = desired_angular_rate[i];

        // Fill in the attitude rate referenced sent to the inner-loop
        statistics_msg_.attitude_rate_reference[i] = attitude_rate_reference[i];
    }

    // Fill in the thrust reference
    statistics_msg_.thrust_reference = thrust_reference;

    // Fill in with the current attitude in degrees
    statistics_msg_.state_roll = Pegasus::Rotations::rad_to_deg(euler_angles[2]);
    statistics_msg_.state_pitch = Pegasus::Rotations::rad_to_deg(euler_angles[1]);
    statistics_msg_.state_yaw = Pegasus::Rotations::rad_to_deg(euler_angles[0]);

    // Fill in with the desired attitude in degrees
    statistics_msg_.desired_roll = Pegasus::Rotations::rad_to_deg(euler_angles_desired[2]);
    statistics_msg_.desired_pitch = Pegasus::Rotations::rad_to_deg(euler_angles_desired[1]);
    statistics_msg_.desired_yaw = Pegasus::Rotations::rad_to_deg(euler_angles_desired[0]);
}

void MellingerController::set_attitude(const Eigen::Vector3d & attitude, double thrust_force, double dt) {

    // Ignore dt
    (void) dt;

    // Set the attitude control message
    attitude_msg_.attitude[0] = attitude[0];
    attitude_msg_.attitude[1] = attitude[1];
    attitude_msg_.attitude[2] = attitude[2];
    attitude_msg_.thrust = thrust_force;

    // Publish the attitude control message for the controller to track
    attitude_publisher_->publish(attitude_msg_);
}

void MellingerController::set_attitude_rate(const Eigen::Vector3d & attitude_rate, double thrust_force, double dt) {

    // Ignore dt
    (void) dt;

    // Set the attitude rate control message
    attitude_rate_msg_.attitude[0] = attitude_rate[0];
    attitude_rate_msg_.attitude[1] = attitude_rate[1];
    attitude_rate_msg_.attitude[2] = attitude_rate[2];
    attitude_rate_msg_.thrust = thrust_force;

    // Publish the attitude rate control message for the controller to track
    attitude_rate_publisher_->publish(attitude_rate_msg_);
}

void MellingerController::reset_controller() {
    // Reset the controllers
    for(unsigned int i=0; i < 3; i++) controllers_[i]->reset_controller();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::MellingerController, autopilot::Controller)