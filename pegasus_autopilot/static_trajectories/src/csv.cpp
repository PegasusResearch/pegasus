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
#include <fstream>
#include "static_trajectories/csv.hpp"

namespace autopilot {

CSVTrajectory::CSVTrajectory(const std::string & filename, const Eigen::Vector3d & offset, bool check_z_negative) {

    // Parse the csv file
    parse_csv(filename);

    // Add the offset to the position
    for (auto &pos : pos_) pos += offset;

    // Check if the z coordinate is always negative or zero (because, in the case of the pegasus, the z axis points down - NED standard)
    if (check_z_negative) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Checking if the z coordinate is always negative or zero in the trajectory. If not, it will be reversed.");
        
        for (int i=0; i < pos_.size(); i++) {
            if (pos_[i](2) > 0.0) {
                // Invert the z-coordinate on the position, velocity, acceleration and jerk
                pos_[i](2) *= -1.0;
                vel_[i](2) *= -1.0;
                acc_[i](2) *= -1.0;
                jerk_[i](2) *= -1.0;
            }
        }
    }

    // Set the maximum gamma to the last time in the trajectory
    max_gamma_ = time_.back();
}

void CSVTrajectory::parse_csv(const std::string & filename) {

    // time, x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, yaw (rad), yaw_rate (rad/s)
    // time,   pos  ,    vel    ,    acc    ,    jerk   , yaw (rad), yaw_rate (rad/s)

    // Create an input filestream
    std::ifstream in(filename);

    // Make sure the file is open
    if(!in.is_open()) throw std::runtime_error("Could not open file");

    // Iterate over the lines of the csv file
    std::string line;

    while(std::getline(in, line)) {

        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Create a vector of all the columns in the line
        std::vector<std::string> row;
        std::string cell;

        while(std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }

        // Check the size of the row
        if (row.size() != 15) throw std::runtime_error("CSV file has wrong number of columns");

        // Convert the row to the correct format
        double time = std::stod(row[0]);
        Eigen::Vector3d pos(std::stod(row[1]), std::stod(row[2]), std::stod(row[3]));
        Eigen::Vector3d vel(std::stod(row[4]), std::stod(row[5]), std::stod(row[6]));
        Eigen::Vector3d acc(std::stod(row[7]), std::stod(row[8]), std::stod(row[9]));
        Eigen::Vector3d jerk(std::stod(row[10]), std::stod(row[11]), std::stod(row[12]));
        double yaw = std::stod(row[13]);
        double yaw_rate = std::stod(row[14]);

        // Append the data to the correct vectors
        time_.push_back(time);
        pos_.push_back(pos);
        vel_.push_back(vel);
        acc_.push_back(acc);
        jerk_.push_back(jerk);
        yaw_.push_back(yaw);
        yaw_rate_.push_back(yaw_rate);
    }

    // Check if all the vectors have the same size
    if (time_.size() != pos_.size() || time_.size() != vel_.size() || time_.size() != acc_.size() || time_.size() != jerk_.size() || time_.size() != yaw_.size() || time_.size() != yaw_rate_.size()) {
        throw std::runtime_error("CSV file has vectors with different sizes. CSV file containing the trajectory is not valid.");
    }

    // Check if the time in the csv file is monotonically increasing
    // Attempt to reverse the vectors if it is not
    if(!std::is_sorted(time_.begin(), time_.end())) {
        std::reverse(time_.begin(), time_.end());
        std::reverse(pos_.begin(), pos_.end());
        std::reverse(vel_.begin(), vel_.end());
        std::reverse(acc_.begin(), acc_.end());
        std::reverse(jerk_.begin(), jerk_.end());
        std::reverse(yaw_.begin(), yaw_.end());
        std::reverse(yaw_rate_.begin(), yaw_rate_.end());
    } 

    // Check if the time in the csv file is monotonically increasing, if it is still not, throw an error
    if(!std::is_sorted(time_.begin(), time_.end())) {
        throw std::runtime_error("CSV file has non-monotonically increasing time. CSV file containing the trajectory is not valid.");
    }

    // Check if the time starts at 0.0
    if (time_[0] != 0.0) throw std::runtime_error("CSV file does not start at time 0.0. CSV file containing the trajectory is not valid.");

    // Get the rate at which the trajectory is sampled
    dt_ = time_[1] - time_[0];
}

int CSVTrajectory::get_closest_index(const double gamma) const {

    // Check if the gamma is within the bounds of the trajectory
    if (gamma < 0.0) return 0;
    if (gamma > time_.back()) return time_.size()-1;

    // Devide the gamma by the time step to get the index
    return std::round(gamma / dt_);
}


Eigen::Vector3d CSVTrajectory::pd(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the position
    return pos_[idx];
}

Eigen::Vector3d CSVTrajectory::d_pd(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Re turn the velocity
    return vel_[idx];
}

Eigen::Vector3d CSVTrajectory::d2_pd(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the acceleration
    return acc_[idx];
}

Eigen::Vector3d CSVTrajectory::d3_pd(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the jerk
    return jerk_[idx];
}


double CSVTrajectory::yaw(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the yaw
    return yaw_[idx];
}

double CSVTrajectory::d_yaw(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the yaw rate
    return yaw_rate_[idx];
}

double CSVTrajectory::vehicle_speed(const double gamma) const {

    // Get the index of the closest time to gamma
    int idx = get_closest_index(gamma);

    // Return the vehicle speed
    return vel_[idx].norm();
}

double CSVTrajectory::vd(const double gamma) const {

    // Return the parametric speed. In this case, it should be 1, has the 
    // time is what parameterizes the csv trajectories and time should progress
    // linearly (unless you are Einstein of course...)
    return 1.0;
}

void CSVFactory::initialize() {

    // Load the service topic from the parameter server
    node_->declare_parameter<std::string>("autopilot.StaticTrajectoryManager.CSVFactory.service", "path/add_csv");

    // Advertise the service to add a line to the path
    add_csv_service_ = node_->create_service<pegasus_msgs::srv::AddCsv>(node_->get_parameter("autopilot.StaticTrajectoryManager.CSVFactory.service").as_string(), std::bind(&CSVFactory::csv_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void CSVFactory::csv_callback(const pegasus_msgs::srv::AddCsv::Request::SharedPtr request, const pegasus_msgs::srv::AddCsv::Response::SharedPtr response) {

    // Log the parameters of the path section to be added
    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding CSV file: " << request->csv_path << " to trajectory. Offset: " << request->offset[0] << "," << request->offset[1] << "," << request->offset[2] << ".");

    // Create a new line
    CSVTrajectory::SharedPtr csv_traj = std::make_shared<CSVTrajectory>(request->csv_path, Eigen::Vector3d(request->offset[0], request->offset[1], request->offset[2]), request->check_z_negative);

    // Add the circle to the path
    this->add_trajectory_to_manager(csv_traj);

    // Set the response to true
    response->success = true;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::CSVFactory, autopilot::StaticTrajectoryFactory)