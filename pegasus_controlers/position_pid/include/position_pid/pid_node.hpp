#pragma once

#include <array>
#include <Eigen/Core>
#include "pid/pid.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pegasus_msgs/msg/path.hpp"
#include "pegasus_msgs/msg/state.hpp"
#include "pegasus_msgs/msg/pid_statistics.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pegasus_msgs/msg/attitude_thrust_control.hpp"

class PidNode : public rclcpp_lifecycle::LifecycleNode {

public:

    /**
     * @brief Constructor for the PID controller node
     * @param node_name The ROS2 node name
     * @param intra_process_comms Whether to use interprocess communication framework or not (false by default)
     */
    PidNode(const std::string & node_name, bool intra_process_comms = false);

    /**
     * @brief Destructor for the PID controller node
     */
    ~PidNode() {}

    /**
     * @defgroup state_machine_callbacks
     * This section defines all the callbacks that are responsible for transitions in the node state machine
     */

    /**
     * @ingroup state_machine_callbacks
     * @brief on_activate callback is being called when the lifecycle node
     * enters the "activating" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "active" state or stays
     * in "inactive".
     * TRANSITION_CALLBACK_SUCCESS transitions to "active"
     * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);

    /**
     * @ingroup state_machine_callbacks
     * @brief on_deactivate callback is being called when the lifecycle node
     * enters the "deactivating" state.
     * Depending on the return value of this function, the state machine
     * either invokes a transition to the "inactive" state or stays
     * in "active".
     * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
     * TRANSITION_CALLBACK_FAILURE transitions to "active"
     * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);

private:

    /**
     * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
     */
    void timer_callback();

    /**
     * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
     * "current_velocity_".
     * @param msg A message with the state of the vehicle
     */
    void update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Method that is called by "path_sub_" to update the variables "desired_position_",
     * "desired_velocity_", "desired_acceleration_" and "desired_yaw_"
     * @param msg A message with the desired path to follow
     */
    void update_reference_callback(const pegasus_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief Method that is called by the "timer_callback" to set the position reference to the current
     * vehicle position. The same applies for the yaw angle. The desired velocity and acceleration
     * are set to zero
     */
    void update_reference_for_hold_position();

    /**
     * @brief The rate at which the timer will call the timer_callback, expressed in Hz
     */
    double timer_rate_;

    /**
     * @brief The publisher for the controller references. By default, a lifecycle publisher is 
     * inactive by creation and has to be activated to publish messages into the ROS world.
     */
    rclcpp_lifecycle::LifecyclePublisher<pegasus_msgs::msg::AttitudeThrustControl>::SharedPtr control_pub_;

    /**
     * @brief The publisher for the statistics for measuring the controller performance.
     */
    rclcpp_lifecycle::LifecyclePublisher<pegasus_msgs::msg::PidStatistics>::SharedPtr statistics_pub_;

    /**
     * @brief The subscriber for the current state of the vehicle.
     */
    rclcpp::Subscription<pegasus_msgs::msg::State>::SharedPtr state_sub_;

    /**
     * @brief The subscriber for the desired references to follow.
     */
    rclcpp::Subscription<pegasus_msgs::msg::Path>::SharedPtr path_sub_;

    /**
     * @brief Control message to be published periodically by the timer_callback() when
     * the controller is running
     */
    pegasus_msgs::msg::AttitudeThrustControl attitude_thrust_msg_;

    /**
     * @brief message to be published periodically by the timer_callback() when the 
     * controller is running 
     */
    pegasus_msgs::msg::PidStatistics statistics_msg_;

    /**
     * @brief Timer used to make the controller run at a constant rate
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief The previous time instant when the controller was called
     */
    rclcpp::Time prev_time_;

    /**
     * @defgroup controller_housekeeping
     * This section defines all variables that are necessarily to actual perform the control
     */

    /**
     * @ingroup controller_housekeeping
     * @brief A pointer to an actual pid controller that will do all the work
     */
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_;
    
    /**
     * @ingroup controller_housekeeping
     * @brief The current position of the vehicle expressed in the inertial frame in NED (x,y,z)
     */
    Eigen::Vector3d current_position_;

    /**
     * @ingroup controller_housekeeping
     * @brief The current velocity of the vehicle expressed in the inertial frame in NED (x_dot, y_dot, z_dot)
     */
    Eigen::Vector3d current_velocity_;

    /**
     * @ingroup controller_housekeeping
     * @brief The current yaw angle, expressed in rad
     */
    double current_yaw_;

    /**
     * @ingroup controller_housekeeping
     * @brief Boolean that checks if we have already received the state the vehicle or not
     */
    bool state_initialized_{false};

    /**
     * @ingroup controller_housekeeping
     * @brief The desired position for the vehicle expressed in the inertial frame in NED
     */
    Eigen::Vector3d desired_position_;

    /**
     * @ingroup controller_housekeeping
     * @brief The desired velocity for the vehicle expressed in the inertial frame in NED (x_dot_des, y_dot_des, z_dot_des)
     */
    Eigen::Vector3d desired_velocity_{0.0, 0.0, 0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The desired acceleration for the vehicle expressed in the inertial frame in ned (x_ddot_des, y_ddot_des, z_ddot_des)
     */
    Eigen::Vector3d desired_acceleration_{0.0, 0.0, 0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The desired yaw angle, expressed in rad
     */
    double desired_yaw_;

    /**
     * @ingroup controller_housekeeping
     * @brief Boolean that checks if we have already received the reference for the vehicle to follow or not
     */
    bool reference_initialized_{false};

    /**
     * @ingroup controller_housekeeping
     * @brief The mass of the vehicle
     */
    double mass_{-1.0};
};