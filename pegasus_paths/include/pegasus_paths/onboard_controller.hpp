#pragma once

#include "base_controller.hpp"
#include "pegasus_msgs/msg/position_control.hpp"

class OnboardController : public BaseControllerNode {

public:

    using SharedPtr = std::shared_ptr<OnboardController>;
    using UniquePtr = std::unique_ptr<OnboardController>;
    using WeakPtr = std::weak_ptr<OnboardController>;

    /**
     * @brief Construct a new Onboard controller
     * @param nh The nodehandler shared pointer for the base class that creates the controller object
     * @param path A shared pointer for a path the controller must track
     * @param controller_rate The rate at which the controller will operate
     */
    OnboardController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate);

    /**
     * @brief Destroy the Onboard controller object
     */
    ~OnboardController();

    /**
     * @brief Method to start the path following controller
     */
    void start();

    /**
     * @brief Method for stoping the path following controller
     */
    void stop();

     /**
     * @brief Method that is called whenever the reference path to follow object is reset. This method should
     * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
     */
    void reset();

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
     * @brief Auxiliary function called inside the "controller_update" to update the references that the controller will
     * track with information from the "path_" object
     */
    void update_references();

    /**
     * @defgroup controller_housekeeping
     * This section defines all the actual variables used for the control of the vehicle
     */

    /**
     * @ingroup controller_housekeeping 
     * @brief The reference position for the PID controller
     */
    Eigen::Vector3d desired_position_{0.0, 0.0, 0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The current parametric value that we are aiming at
     */
    double gamma_{0.0};

    /**
     * @ingroup controller_housekeeping
     * @brief The speed progression that we are aiming at
     */
    double gamma_dot_{0.0};

    /**
     * @defgroup ros_messages_callbacks_publishers Controller ROS2 messages, subscriber callbacks and publishers
     */

    /**
     * @ingroup ros_messages_callbacks_publishers
     * @brief Control message to be published periodically by the timer_callback() when
     * the controller is running
     */
    pegasus_msgs::msg::PositionControl position_msg_;

    /**
     * @brief Publisher for the desired position references for the onboard controller to track
     */
    rclcpp::Publisher<pegasus_msgs::msg::PositionControl>::SharedPtr control_pub_{nullptr};

};