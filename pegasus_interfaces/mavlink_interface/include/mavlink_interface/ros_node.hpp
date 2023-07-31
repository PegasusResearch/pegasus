#pragma once

#include "rclcpp/rclcpp.hpp"

#include "mavlink_node.hpp"
#include "thrust_curves/thrust_curves.hpp"

// Messages for the sensor data (IMU, barometer, GPS, etc.)
#include "sensor_msgs/msg/imu.hpp"
#include "pegasus_msgs/msg/sensor_barometer.hpp"
#include "pegasus_msgs/msg/sensor_gps.hpp"
#include "pegasus_msgs/msg/sensor_gps_info.hpp"

// Messages for the state of the vehicle (pose, velocity, angular velocity, etc. provided by EKF)
#include "nav_msgs/msg/odometry.hpp"
#include "pegasus_msgs/msg/rpy.hpp"

// Messages for the current status of the vehicle (armed, landed, etc.)
#include "pegasus_msgs/msg/status.hpp"

// Messages for the control commands (position, attitude, etc.)
#include "pegasus_msgs/msg/control_position.hpp"
#include "pegasus_msgs/msg/control_attitude.hpp"

// Services for arming, auto-landing, etc.
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/land.hpp"

// Messages for the mocap fusion and visual odometry
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <mavsdk/plugins/telemetry/telemetry.h>

/**
 * @brief 
 */
class ROSNode : public rclcpp::Node {

public:

    /**
     * @brief Constructor for the ROS2 node that receives control commands 
     * from ROS and send sensor data to ROS2
     */
    ROSNode();
    ~ROSNode();

    /**
     * @brief Method used to start the ROS2 node in one thread and mavsdk in another thread
     */
    void start();

    /**
     * @defgroup initFunctions 
     * This group defines all the private initialization functions
     * that will initialize all the ROS publishers, subscribers, services and actions
     */

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 parameters that we are supposed to read
     * from the parameter server
     */
    void init_parameters();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 publishers
     */
    void init_publishers();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 subscribers
     */
    void init_subscribers_and_services();

    /**
     * @ingroup initFunctions
     * @brief Method that is called to update the system_id field in the status_msg. This method
     * DOES NOT publish the most up to date message to the status_pub
     * @param id An unsigned int (a byte) that encodes the mavlink id of the connected vehicle
     */
    void update_system_id(const uint8_t & id);

    /**
     * @defgroup publisherMessageUpdate
     * This group defines all the methods used to update the messages which encode the status/state of the vehicle
     * and that will be published to ROS2 via a publisher
     */

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the imu_msg. This method publishes
     * the most up to date message to imu_pub.
     * @param imu A mavsdk structure which contains:
     * - the angular velocity measured by the imu (in rad)
     * - the linear acceleration measured by the imu (in m/s^2)
     */
    void on_imu_callback(const mavsdk::Telemetry::Imu &imu);
    void on_altitude_callback(const mavsdk::Telemetry::Altitude &altitude);
    void on_raw_gps_callback(const mavsdk::Telemetry::RawGps &gps);
    void on_gps_info_callback(const mavsdk::Telemetry::GpsInfo &gps_info);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the pose.orientation field in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param quat A mavsdk structure which contains a quaternion encoding the attitude of the vehicle in NED
     */
    void on_quaternion_callback(const mavsdk::Telemetry::Quaternion &quat);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the body_vel.twist field in the state_msg. This method 
     * publishes the most up to date message to state_pub
     * @param ang_vel A mavsdk structure which contains the angular velocity of the vehicle expressed in the body frame
     * according to the f.r.d frame
     */
    void on_angular_velocity_callback(const mavsdk::Telemetry::AngularVelocityBody &ang_vel);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the pose and inertial_vel fields in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param pos_vel_ned A mavsdk structure which contains the position and linear velocity of the vehicle expressed in the inertial frame
     * in NED
     */
    void on_position_velocity_callback(const mavsdk::Telemetry::PositionVelocityNed &pos_vel_ned);    

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the armed field in the status_msg. This method
     * publishes the most up to date message to the status_pub
     * @param is_armed A boolean whether the vehicle is armed(1)/disarmed(0)
     */
    void on_armed_callback(const bool & is_armed);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the landed_state field in the status_msg. It is an enum, which
     * can assume the following values:
     * - 0 - UKNOWN
     * - 1 - ON_GROUND
     * - 2 - IN_AIR
     * - 3 - TAKING_OFF
     * - 4 - LANDING
     * @param landed_state A mavsdk structure wich contains the high level state of the vehicle
     */
    void on_landed_state_callback(const mavsdk::Telemetry::LandedState & landed_state);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the flight_mode field in the status_msg. This method
     * publishes the most up to date message to the status_pub
     * @param flight_mode An byte that containts the current flightmode active in the onboard microcontroller
     */
    void on_flight_mode_callback(const mavsdk::Telemetry::FlightMode & flight_mode);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the health field in the status_msg. Contains the variables:
     * - is_armable
     * - accelerometer_calibrated
     * - magnetometer_calibrated
     * - local_position_ok
     * - global_position_ok
     * - home_position_ok
     * This method publishes the most up to date message to status_pub
     * @param health A mavsdk structure which containts the state of the above vehicle variables
     */
    void on_health_callback(const mavsdk::Telemetry::Health & health);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the remaining battery percentage field in the status_msg. This method
     * publishes the most up to date message to status_pub
     * @param percentage A float with the current battery percentage
     */
    void on_battery_callback(const mavsdk::Telemetry::Battery & battery);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the signal strength of a connected vehicle RC controller field in the status_msg. This method
     * publishes the most up to date message to status_pub
     * @param rc_signal_strength_percentage A float with the current rc signal strength percentage. Negative value means no RC connected
     */
    void on_rc_callback(const mavsdk::Telemetry::RcStatus & rc_signal);

    /**
     * @defgroup dataGetters
     * This group defines all the methods used to get data from the current state of the vehicle
     */

    /**
     * @ingroup dataGetters
     * @brief Get the current flight mode active in the vehicle
     * @return const unsigned char& a reference to the current active flight mode (an unsigned int)
     */
    inline const unsigned char& get_current_flight_mode() {
        return status_msg_.flight_mode;
    }

    /**
     * @ingroup dataGetters
     * @brief Get the offboard standard code
     * @return const unsigned& a reference to the offboard mode
     */
    inline const unsigned char& get_offboard_code_mode() {
        return status_msg_.OFFBOARD;
    }

private:

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
     * @param response The response in this service goes empty, as the request is done asynchronously through mavlink
     */
    void arm_callback(const pegasus_msgs::srv::Arm::Request::SharedPtr request, const pegasus_msgs::srv::Arm::Response::SharedPtr response);
   
    /**
     * @ingroup servicesCallbacks
     * @brief Autoland service callback. When a service request is reached from the land_service_,
     * this callback is called and will send a mavlink command for the vehicle to autoland using the onboard controller
     * @param request An empty request for landing the vehicle (can be ignored)
     * @param response The response in this service goes empty, as the request id done asynchronously through mavlink
     */
    void land_callback(const pegasus_msgs::srv::Land::Request::SharedPtr request, const pegasus_msgs::srv::Land::Response::SharedPtr response);

    /**
     *  @defgroup messages 
     *  This group defines all the ROS messages that will always be constant and
     *  updated with the most recent values
     */

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

    /**
     *  @defgroup publishers ROS2 Publishers
     *  This group defines all the ROS publishers
     */

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

    /**
     * @defgroup subscribers ROS2 Subscribers
     * This group defines all the ROS subscribers
     */

    /**
     * @ingroup subscribers
     * @brief Position subscriber. The position of the vehicle should be expressed in the NED reference frame
     */
    rclcpp::Subscription<pegasus_msgs::msg::ControlPosition>::SharedPtr position_control_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude and thrust subscriber. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     */
    rclcpp::Subscription<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_thrust_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude rate and thrust subscriber. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     */
    rclcpp::Subscription<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_thrust_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude and force subscriber. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The force along the body Z-axis 
     * should be specified in Newton (N)
     */
    rclcpp::Subscription<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_force_sub_{nullptr};
    
    /**
     * @ingroup subscribers
     * @brief Attitude rate and thrust subscriber. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The force along the body Z-axis
     * should be specified in Newton (N)
     */
    rclcpp::Subscription<pegasus_msgs::msg::ControlAttitude>::SharedPtr attitude_rate_force_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Subscriber for a position of the vehicle yielded by a Motion Capture System, if available. This
     * subscriber should receive the information expressed in the ENU reference frame
     */
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_enu_sub_{nullptr};

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
     * @brief Service server to auto-land the vehicle using the 
     * microcontroller embeded control algorithm
     */
    rclcpp::Service<pegasus_msgs::srv::Land>::SharedPtr land_service_{nullptr};

    /**
     * @brief A MavlinkNode object that allows for initializing the ROS2 publishers, subscribers, etc.
     */
    MavlinkNode::UniquePtr mavlink_node_{nullptr};
    MavlinkNode::MavlinkNodeConfig mavlink_config_;

    /**
     * @brief Thrust curve object used to set the conversion from thrust in Newton (N) to percentage
     * which is then sent to the mavlink onboard controller (initialized by the init_thrust_curve) which will
     * read this parameter from the ROS parameter server
     */
    std::shared_ptr<Pegasus::ThrustCurve> thrust_curve_{nullptr};

    // ROS 2 thread setup
    rclcpp::executors::MultiThreadedExecutor executor_;
};
