#pragma once

#include "pid/pid.hpp"
#include "controller.hpp"
#include "pegasus_msgs/msg/pid_statistics.hpp"
#include "pegasus_msgs/msg/attitude_thrust_control.hpp"

class PidController : public BaseControllerNode {

public:

    using SharedPtr = std::shared_ptr<PidController>;
    using UniquePtr = std::unique_ptr<PidController>;
    using WeakPtr = std::weak_ptr<PidController>;

    /**
     * @brief Construct a new Pid Controller object
     * @param nh The nodehandler shared pointer for the base class that creates the controller object
     * @param path A shared pointer for a path the controller must track
     * @param controller_rate The rate at which the controller will operate
     */
    PidController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate);

    /**
     * @brief Destroy the Pid Controller object
     */
    ~PidController();

    /**
     * @brief Method to start the path following controller
     */
    void start();

    /**
     * @brief Method for stoping the path following controller
     */
    void stop();

    /**
     * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
     * "current_velocity_".
     * @param msg A message with the state of the vehicle
     */
    void update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
     * which is used to update the control signals
     */
    void controller_update();

private:

    /**
     * @brief Method that is called by the "timer_callback" to fill the PID statistics for the x, y and z position pids
     */
    void update_statistics_msg();

    /**
     * @defgroup controller_housekeeping
     * This section defines all the actual variables used for the control of the vehicle
     */

    /**
     * @ingroup controller_housekeeping
     * @brief A pointer to an actual pid controller that will do all the work
     */
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_{nullptr};

    /**
     * @ingroup controller_housekeeping
     * @brief The current position of the vehicle expressed in the inertial frame in NED (x,y,z)
     */
    Eigen::Vector3d current_position_{0.0, 0.0, 0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The current velocity of the vehicle expressed in the inertial frame in NED (x_dot, y_dot, z_dot)
     */
    Eigen::Vector3d current_velocity_{0.0, 0.0, 0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The current yaw angle, expressed in rad
     */
    double current_yaw_{0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The desired yaw angle, expressed in rad
     */
    double desired_yaw_;

    /**
     * @ingroup controller_housekeeping
     * @brief The mass of the vehicle
     */
    double mass_{-1.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The current parametric value that we are aiming at
     */
    double gamma_{0.0};

    /**
     * @defgroup ros_messages_callbacks_publishers Controller ROS2 messages, subscriber callbacks and publishers
     */

    /**
     * @ingroup ros_messages_callbacks_publishers
     * @brief Control message to be published periodically by the timer_callback() when
     * the controller is running
     */
    pegasus_msgs::msg::AttitudeThrustControl attitude_thrust_msg_;

    /**
     * @ingroup ros_messages_callbacks_publishers
     * @brief message to be published periodically by the timer_callback() when the 
     * controller is running 
     */
    pegasus_msgs::msg::PidStatistics statistics_msg_;

    /**
     * @brief Publisher for the attitute and total thrust force (in N) using the "attitude_thrust_msg_"
     */
    rclcpp::Publisher<pegasus_msgs::msg::AttitudeThrustControl>::SharedPtr control_pub_{nullptr};
    
    /**
     * @brief Publisher for the statistics of the pid position controller
     */
    rclcpp::Publisher<pegasus_msgs::msg::PidStatistics>::SharedPtr statistics_pub_{nullptr};
};