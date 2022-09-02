#pragma once
#include "paths/dense.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pegasus_msgs/msg/state.hpp"
#include "pegasus_utils/rotations.hpp"

class BaseControllerNode { 

public: 

    using SharedPtr = std::shared_ptr<BaseControllerNode>;
    using UniquePtr = std::unique_ptr<BaseControllerNode>;
    using WeakPtr = std::weak_ptr<BaseControllerNode>;

    /**
     * @brief Virtual destructor for the base controller node
     */
    virtual ~BaseControllerNode();

    /**
     * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
     * which is used to update the control signals
     */
    virtual void controller_update() = 0;

    /**
     * @brief Method to start the path following controller
     */
    virtual void start() {
        // Update the last time that the controller was called to now (initialization)
        prev_time_ = nh_->get_clock()->now();

        // Initialize the periodic timer
        timer_ = nh_->create_wall_timer(std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&BaseControllerNode::controller_update, this));
    }

    /**
     * @brief Method for stoping the path following controller
     */
    virtual void stop() {
        // Reset the timer that calls the control law periodically
        timer_->cancel();
        timer_->reset();
    }

protected:
    
    /**
     * @brief Construct a new Controller Node object which implements a given type of controller
     * to follow a specific path
     * @param nh The nodehandler shared pointer for the base class that creates the controller object
     * @param path A shared pointer for a path the controller must track
     * @param controller_rate The rate at which the controller will operate
     */
    BaseControllerNode(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate) : nh_(nh), path_(path), timer_rate_(controller_rate) {
        // Initialize the subscriber to the state of the vehicle
        nh_->declare_parameter<std::string>("controller_node.topics.subscribers.state", "nav/state");
        state_sub_ = nh_->create_subscription<pegasus_msgs::msg::State>(nh_->get_parameter("controller_node.topics.subscribers.state").as_string(), 1, std::bind(&BaseControllerNode::update_state_callback, this, std::placeholders::_1));
    }

    /**
     * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
     * "current_velocity_".
     * @param msg A message with the state of the vehicle
     */
    virtual void update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg) { 
        
        // Save the pointer to the complete state message
        state_ = msg; 
    }

    /**
     * @brief A pointer to the base class node handler
     */
    rclcpp::Node::SharedPtr nh_{nullptr};

    /**
     * @brief The subscriber for the current state of the vehicle.
     */
    rclcpp::Subscription<pegasus_msgs::msg::State>::SharedPtr state_sub_;

    /**
     * @brief Timer used to make the controller run at a constant rate
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief The previous time instant when the controller was called
     */
    rclcpp::Time prev_time_;
    
    /**
     * @brief The rate at which the timer will call the timer_callback, expressed in Hz
     */
    double timer_rate_;

    /**
     * @brief The path that the controller has to track
     */
    Pegasus::Paths::Path::SharedPtr path_{nullptr};

    /**
     * @brief The current vehicle state
     */
    pegasus_msgs::msg::State::SharedPtr state_{nullptr};
};