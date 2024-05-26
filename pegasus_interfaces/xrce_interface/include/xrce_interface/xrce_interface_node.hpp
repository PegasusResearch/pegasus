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

#include "rclcpp/rclcpp.hpp"
#include "thrust_curves/thrust_curves.hpp"

// Messages for the sensor data (IMU, barometer, GPS, etc.)
#include "sensor_msgs/msg/imu.hpp"
#include "pegasus_msgs/msg/sensor_barometer.hpp"
#include "pegasus_msgs/msg/sensor_gps.hpp"
#include "pegasus_msgs/msg/sensor_gps_info.hpp"

// Messages for the state of the vehicle (pose, velocity, angular velocity, etc. provided by EKF)
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/rpy.hpp"

// Messages for the current status of the vehicle (armed, landed, etc.) and the vehicle constants such as mass and thrust curve
#include "pegasus_msgs/msg/status.hpp"
#include "pegasus_msgs/msg/vehicle_constants.hpp"

// Messages for the control commands (position, attitude, etc.)
#include "pegasus_msgs/msg/control_position.hpp"
#include "pegasus_msgs/msg/control_attitude.hpp"

// Services for arming, auto-landing, etc.
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/kill_switch.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/srv/offboard.hpp"
#include "pegasus_msgs/srv/position_hold.hpp"

// Messages for the mocap fusion and visual odometry
#include "geometry_msgs/msg/pose_stamped.hpp"

class XRCEInterfaceNode : public rclcpp::Node {

public:
    XRCEInterfaceNode();
    ~XRCEInterfaceNode();

private:

    void init_parameters();
    void initialize_publishers();
    void intialize_subscribers();
    void initialize_services();
    void init_thrust_curve();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize the ThrustCurve object used
     * to translate the thrust curve from Newton (N) to percentage (0-100%) and vice-versa
     * based on the configurations loaded from ROS parameter server
     */
    void init_thrust_curve();

    /**
     * @defgroup subscriberCallbacks
     * This group defines all the ROS subscriber callbacks
     */

    /**
     * @ingroup subscriberCallbacks
     * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
     * @param msg A message with the desired position for the vehicle in NED
     */
    void position_callback(const pegasus_msgs::msg::ControlPosition::ConstSharedPtr msg);
    
    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     * @param msg A message with the desired attitude and thrust to apply to the vehicle
     */
    void attitude_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
     */
    void attitude_rate_thrust_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
     * the body Z-axis should be given in Newton (N)
     * @param msg A message with the desired attitude and force to apply to the vehicle
     */
    void attitude_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
     * the body Z-axis should be given in Newton (N)
     * @param msg A message with the desired attitude-rate and force to apply to the vehicle
     */
    void attitude_rate_force_callback(const pegasus_msgs::msg::ControlAttitude::ConstSharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
     * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
     * sends it via mavlink to the vehicle autopilot filter to merge
     * @param msg A message with the pose of the vehicle expressed in ENU
     */
    void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    /**
     * @defgroup servicesCallbacks
     * This group defines all the service server callbacks,
     * such as arming/disarming the vehicle or auto-landing
     */

    /**
     * @ingroup servicesCallbacks
     * @brief Arming/disarming service callback. When a service request is reached from the arm_service_, 
     * this callback is called and will send a mavlink command for the vehicle to arm/disarm
     * @param request The request for arming (bool = true) or disarming (bool = false)
     * @param response The response in this service uint8
     */
    void arm_callback(const pegasus_msgs::srv::Arm::Request::SharedPtr request, const pegasus_msgs::srv::Arm::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Kill switch service callback. When a service request is reached from the kill_switch_service_,
     * this callback is called and will send a mavlink command for the vehicle to kill the motors instantly.
     * @param request The request for killing the motors (bool = true)
     * @param response The response in this service uint8
    */
    void kill_switch_callback(const pegasus_msgs::srv::KillSwitch::Request::SharedPtr request, const pegasus_msgs::srv::KillSwitch::Response::SharedPtr response);
   
    /**
     * @ingroup servicesCallbacks
     * @brief Autoland service callback. When a service request is reached from the land_service_,
     * this callback is called and will send a mavlink command for the vehicle to autoland using the onboard controller
     * @param request An empty request for landing the vehicle (can be ignored)
     * @param response The response in this service uint8
     */
    void land_callback(const pegasus_msgs::srv::Land::Request::SharedPtr request, const pegasus_msgs::srv::Land::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Offboard service callback. When a service request is reached from the offboard_service_,
     * this callback is called and will send a mavlink command for the vehicle to enter offboard mode
     * @param request An empty request for entering offboard mode (can be ignored)
     * @param response The response in this service uint8
     */
    void offboard_callback(const pegasus_msgs::srv::Offboard::Request::SharedPtr, const pegasus_msgs::srv::Offboard::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Position hold service callback. When a service request is reached from the position_hold_service_,
     * this callback is called and will send a mavlink command for the vehicle to enter position hold mode
     * @param request An empty request for entering position hold mode (can be ignored)
     * @param response The response in this service uint8
     */
    void position_hold_callback(const pegasus_msgs::srv::PositionHold::Request::SharedPtr, const pegasus_msgs::srv::PositionHold::Response::SharedPtr response);
    
    /**
     * @ingroup messages
     * Messages corresponding to the sensors of the vehicles
     */
    sensor_msgs::msg::Imu imu_msg_;
    pegasus_msgs::msg::SensorBarometer baro_msg_;
    pegasus_msgs::msg::SensorGps gps_msg_;
    pegasus_msgs::msg::SensorGpsInfo gps_info_msg_;

    /**
     * @ingroup messages
     * Message corresponding to the filtered state of the vehicle (from internal EKF) */
    nav_msgs::msg::Odometry filter_state_msg_;
    pegasus_msgs::msg::RPY filter_state_rpy_msg_;

    /**
     * @ingroup messages
     * Message corresponding to the status of the vehicle */
    pegasus_msgs::msg::Status status_msg_;
    pegasus_msgs::msg::VehicleConstants vehicle_constants_msg_;

    /**
     * @ingroup publishers 
     * @brief FMU sensors publishers 
     */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::SensorBarometer>::SharedPtr baro_pub_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::SensorGps>::SharedPtr gps_pub_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::SensorGpsInfo>::SharedPtr gps_info_pub_{nullptr};
    
    /**
     * @ingroup publishers
     * @brief FMU EKF filter state
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filter_state_pub_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::RPY>::SharedPtr filter_state_rpy_pub_{nullptr};

    /**
     * @ingroup publishers
     * @brief 
     */
    rclcpp::Publisher<pegasus_msgs::msg::Status>::SharedPtr status_pub_{nullptr};
    rclcpp::Publisher<pegasus_msgs::msg::VehicleConstants>::SharedPtr vehicle_constants_pub_{nullptr};

    /**
     * @defgroup services ROS2 Services
     * This group defines all the ROS services
     */

    /**
     * @ingroup services
     * @brief Service server to arm the vehicle
     */
    rclcpp::Service<pegasus_msgs::srv::Arm>::SharedPtr arm_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to disarm the vehicle
     */
    rclcpp::Service<pegasus_msgs::srv::KillSwitch>::SharedPtr kill_switch_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to auto-land the vehicle using the 
     * microcontroller embeded control algorithm
     */
    rclcpp::Service<pegasus_msgs::srv::Land>::SharedPtr land_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to set the vehicle into the offboard mode 
     */
    rclcpp::Service<pegasus_msgs::srv::Offboard>::SharedPtr offboard_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to set the vehicle into the hold position mode
     */
    rclcpp::Service<pegasus_msgs::srv::PositionHold>::SharedPtr position_hold_service_{nullptr};

    /**
     * @brief Thrust curve object used to set the conversion from thrust in Newton (N) to percentage
     * which is then sent to the mavlink onboard controller (initialized by the init_thrust_curve) which will
     * read this parameter from the ROS parameter server
     */
    std::shared_ptr<Pegasus::ThrustCurve> thrust_curve_{nullptr};
};