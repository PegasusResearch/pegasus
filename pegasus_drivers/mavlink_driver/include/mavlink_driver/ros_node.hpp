#pragma once

#include "ros_node.fwd.hpp"
#include "mavlink_node.fwd.hpp"
#include "thrust_curves/thrust_curves.hpp"

#include "rclcpp/rclcpp.hpp"

#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/srv/reboot.hpp"
#include "pegasus_msgs/srv/thrust_curve.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "pegasus_msgs/msg/state.hpp"
#include "pegasus_msgs/msg/status.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "pegasus_msgs/msg/position_control.hpp"
#include "pegasus_msgs/msg/body_velocity_control.hpp"
#include "pegasus_msgs/msg/attitude_thrust_control.hpp"
#include "pegasus_msgs/msg/attitude_rate_thrust_control.hpp"

#include <mavsdk/plugins/telemetry/telemetry.h>

/**
 * @brief 
 */
class ROSNode {

public:

    /**
     * @brief Constructor for the ROS2 node that receives control commands 
     * from ROS and send sensor data to ROS2
     * @param nh The ROS2 nodehandler necessary to read parameters from the node parameter server
     */
    ROSNode(rclcpp::Node::SharedPtr nh);

    /**
     * @brief Destroy the ROSNode object
     */
    ~ROSNode();    

    /**
     * @brief Method used to initialize the mavlink_node object pointer
     * @param mavlink_node A pointer to a mavlink_node object
     */
    void init_mavlink_node(const std::shared_ptr<MavlinkNode> mavlink_node);

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
    void init_dynamic_parameters();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 publishers
     */
    void init_publishers();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 subscribers
     */
    void init_subscribers();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 services
     */
    void init_services();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 actions
     */
    void init_actions();

    /**
     * @defgroup publisherMessageUpdate
     * This group defines all the methods used to update the messages which encode the status/state of the vehicle
     * and that will be published to ROS2 via a publisher
     */

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the system_id field in the status_msg. This method
     * DOES NOT publish the most up to date message to the status_pub
     * @param id An unsigned int (a byte) that encodes the mavlink id of the connected vehicle
     */
    void update_system_id(const uint8_t &id);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the armed field in the status_msg. This method
     * publishes the most up to date message to the status_pub
     * @param is_armed A boolean whether the vehicle is armed(1)/disarmed(0)
     */
    void update_armed_state(const bool &is_armed);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the flight_mode field in the status_msg. This method
     * publishes the most up to date message to the status_pub
     * @param flight_mode An byte that containts the current flightmode active in the onboard microcontroller
     */
    void update_flightmode_state(const unsigned char &flight_mode);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the remaining battery percentage field in the status_msg. This method
     * publishes the most up to date message to status_pub
     * @param percentage A float with the current battery percentage
     */
    void update_battery_state(const float &percentage);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the signal strength of a connected vehicle RC controller field in the status_msg. This method
     * publishes the most up to date message to status_pub
     * @param rc_signal_strength_percentage A float with the current rc signal strength percentage. Negative value means no RC connected
     */
    void update_rc_state(const float &rc_signal_strength_percentage);

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
    void update_heath_state(const mavsdk::Telemetry::Health &health);

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
    void update_landed_state(const mavsdk::Telemetry::LandedState &landed_state);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the pose and inertial_vel fields in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param pos_vel_ned A mavsdk structure which contains the position and linear velocity of the vehicle expressed in the inertial frame
     * in NED
     */
    void update_position_velocity_ned(const mavsdk::Telemetry::PositionVelocityNed &pos_vel_ned);
    
    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the body_vel field in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param odom A mavsdk structure which contains vehicle odometry data expressed in NED, from which
     * we only care about the body velocity
     */
    void update_odometry(const mavsdk::Telemetry::Odometry &odom);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the pose.orientation field in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param quat A mavsdk structure which contains a quaternion encoding the attitude of the vehicle in NED
     */
    void update_attitude_quaternion_ned(const mavsdk::Telemetry::Quaternion &quat);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the body_vel.twist field in the state_msg. This method 
     * publishes the most up to date message to state_pub
     * @param ang_vel A mavsdk structure which contains the angular velocity of the vehicle expressed in the body frame
     * according to the f.r.d frame
     */
    void update_angular_velocity_body(const mavsdk::Telemetry::AngularVelocityBody &ang_vel);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the rpy field in the state_msg. This method
     * publishes the most up to date message to state_pub
     * @param euler A mavsdk structure which contains the euler angles representing the attitude of the vehicle
     * with a body frame defined according to f.r.d relative to a NED inertial frame, expressed in the inertial frame, 
     * according to a Z-Y-X rotation
     */
    void update_attitude_ned_euler(const mavsdk::Telemetry::EulerAngle &euler);

    /**
     * @ingroup publisherMessageUpdate
     * @brief Method that is called to update the imu_msg and mag_msg_. This method publishes
     * the most up to date message to imu_pub and mag_pub.
     * @param imu A mavsdk structure which contains:
     * - the angular velocity measured by the imu (in rad)
     * - the linear acceleration measured by the imu (in m/s^2)
     * - the magnetic field (in gauss)
     */
    void update_imu_state(const mavsdk::Telemetry::Imu &imu);

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
     * @ingroup initFunctions
     * @brief Method used to initialize the mass of the vehicle (loaded from ROS parameter server)
     */
    void init_mass();

    /**
     * @defgroup subscriberCallbacks
     * This group defines all the ROS subscriber callbacks
     */
    
    /**
     * @ingroup subscriberCallbacks
     * @brief Body Velocity subscriber callback. For multirotors, the x,y,z axis on the body frame are
     * always assumed by the autopilot to not be affected by roll and pitch, wich means that
     * they lie in a constant plane (otherwise, controlling in these variables would lead the drone to crash)
     * @param msg A message with the desired body velocity to apply to the vehicle
     */
    void body_velocity_callback(const pegasus_msgs::msg::BodyVelocityControl::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Position subscriber callback. The position of the vehicle should be expressed in the NED reference frame
     * @param msg A message with the desired position for the vehicle in NED
     */
    void position_callback(const pegasus_msgs::msg::PositionControl::SharedPtr msg);
    
    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     * @param msg A message with the desired attitude and thrust to apply to the vehicle
     */
    void attitude_thrust_callback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     * @param msg A message with the desired attitude-rate and thrust to apply to the vehicle
     */
    void attitude_rate_thrust_callback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude and thrust subscriber callback. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
     * the body Z-axis should be given in Newton (N)
     * @param msg A message with the desired attitude and force to apply to the vehicle
     */
    void attitude_force_callback(const pegasus_msgs::msg::AttitudeThrustControl::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Attitude rate and thrust subscriber callback. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The total force along
     * the body Z-axis should be given in Newton (N)
     * @param msg A message with the desired attitude-rate and force to apply to the vehicle
     */
    void attitude_rate_force_callback(const pegasus_msgs::msg::AttitudeRateThrustControl::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Motion Capture vehicle pose subscriber callback. This callback receives a message with the pose of the vehicle
     * provided by a Motion Capture System (if available) expressed in ENU reference frame, converts to NED and 
     * sends it via mavlink to the vehicle autopilot filter to merge
     * @param msg A message with the pose of the vehicle expressed in ENU
     */
    void mocap_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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
     * @ingroup servicesCallbacks
     * @brief Reboot service callback. When a service request is reached from the reboot_service_,
     * this callback will send a mavlink command for the vehicle's microcontroller to reboot (not the onboard PC!)
     * @param request An empty request for rebooting the vehicle (can be ignored)
     * @param response The response in this service goes empty, as the request id done asynchronously through mavlink
     */
    void reboot_callback(const pegasus_msgs::srv::Reboot::Request::SharedPtr request, const pegasus_msgs::srv::Reboot::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Service that is avaialable for any node to request the current thrust curve of the active drone
     * as well as the mass of the vehicle (for control purposes)
     * @param request An empty request (can be ignored)
     * @param response The response in this empty contains a string with the identifier of the type of thrust curve
     * used for the computations (Quadratic, Linear, etc.), the thrust curve parameter names, the actual parameter values and the mass of the vehicle
     */
    void thrust_curve_callback(const pegasus_msgs::srv::ThrustCurve::Request::SharedPtr request, const pegasus_msgs::srv::ThrustCurve::Response::SharedPtr response);

    /**
     *  @defgroup messages 
     *  This group defines all the ROS messages that will always be constant and
     *  updated with the most recent values
     */

    /**
     * @ingroup messages
     * @brief ROS message that holds the current status of the vehicle, such as battery,
     * armed/disarmed state, etc.
     */
    pegasus_msgs::msg::Status status_msg_;
        
    /**
     * @ingroup messages
     * @brief Messages for the state of the vehicle, received by the EKF
     */
    pegasus_msgs::msg::State state_msg_;

    /**
     * @ingroup messages
     * @brief Messages received directly from the IMU
     */
    sensor_msgs::msg::Imu imu_msg_;

    /**
     * @ingroup messages
     * @brief Messages of the magnetic field measured by the IMU
     */
    sensor_msgs::msg::MagneticField mag_msg_;

    /**
     *  @defgroup publishers ROS2 Publishers
     *  This group defines all the ROS publishers
     */

    /**
     * @ingroup publishers 
     * @brief Vehicle Status publisher (such as battery level, armed/disarmed state, etc.)
     */
    rclcpp::Publisher<pegasus_msgs::msg::Status>::SharedPtr status_pub_{nullptr};

    /**
     * @ingroup publishers
     * @brief Vehicle State publisher (position, orientation, velocity, orientation-rate, etc.)
     */
    rclcpp::Publisher<pegasus_msgs::msg::State>::SharedPtr state_pub_{nullptr};
    
    /**
     * @ingroup publishers
     * @brief IMU raw data and measurements publisher
     */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};

    /**
     * @ingroup publishers
     * @brief Magnetometer raw data publisher
     */
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_{nullptr};

    /**
     * @defgroup subscribers ROS2 Subscribers
     * This group defines all the ROS subscribers
     */

    /**
     * @ingroup subscribers
     * @brief Body Velocity subscriber. For multirotors, the x,y,z axis on the body frame are
     * always assumed by the autopilot to not be affected by roll and pitch, wich means that
     * they lie in a constant plane (otherwise, controlling in these variables would lead the drone to crash)
     */
    rclcpp::Subscription<pegasus_msgs::msg::BodyVelocityControl>::SharedPtr body_velocity_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Position subscriber. The position of the vehicle should be expressed in the NED reference frame
     */
    rclcpp::Subscription<pegasus_msgs::msg::PositionControl>::SharedPtr position_control_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude and thrust subscriber. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     */
    rclcpp::Subscription<pegasus_msgs::msg::AttitudeThrustControl>::SharedPtr attitude_thrust_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude rate and thrust subscriber. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The thrust should be normalized
     * between 0-100 %
     */
    rclcpp::Subscription<pegasus_msgs::msg::AttitudeRateThrustControl>::SharedPtr attitude_rate_thrust_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Attitude and force subscriber. The attitude should be specified in euler angles in degrees
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The force along the body Z-axis 
     * should be specified in Newton (N)
     */
    rclcpp::Subscription<pegasus_msgs::msg::AttitudeThrustControl>::SharedPtr attitude_force_sub_{nullptr};
    
    /**
     * @ingroup subscribers
     * @brief Attitude rate and thrust subscriber. The attitude-rate should be specified in euler angles in degrees-per-second
     * according to the Z-Y-X convention in the body frame of f.r.d convention. The force along the body Z-axis
     * should be specified in Newton (N)
     */
    rclcpp::Subscription<pegasus_msgs::msg::AttitudeRateThrustControl>::SharedPtr attitude_rate_force_sub_{nullptr};

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
     * @ingroup services
     * @brief Service server to reboot the vehicle microcontroller
     * via mavlink
     */
    rclcpp::Service<pegasus_msgs::srv::Reboot>::SharedPtr reboot_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to request the thrust curve and mass of the vehicle being controlled
     */
    rclcpp::Service<pegasus_msgs::srv::ThrustCurve>::SharedPtr thrust_curve_service_{nullptr};

    /**
     * @brief A ROS2 nodehandler that allows for reading parameters 
     * from the parameter server, along publishing to ROS2 topics
     */
    rclcpp::Node::SharedPtr nh_{nullptr};

    /**
     * @brief A MavlinkNode object that allows for initializing the ROS2 publishers, subscribers, etc.
     */
    std::shared_ptr<MavlinkNode> mavlink_node_{nullptr};

    /**
     * @brief Thrust curve object used to set the conversion from thrust in Newton (N) to percentage
     * which is then sent to the mavlink onboard controller (initialized by the init_thrust_curve) which will
     * read this parameter from the ROS parameter server
     */
    std::shared_ptr<Pegasus::ThrustCurve> thrust_curve_{nullptr};

    /**
     * @brief The mass of the vehicle (initialized by the init_mass method) which will
     * read this parameter from the ROS parameter server
     */
    double mass_{0.0};


};