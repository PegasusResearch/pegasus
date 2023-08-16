#pragma once

#include <functional>
#include <Eigen/Core>

// ROS imports
#include "rclcpp/rclcpp.hpp"

// Pegasus imports
#include "state.hpp"

namespace autopilot {

class Mode {

public:

    using SharedPtr = std::shared_ptr<Mode>;
    using UniquePtr = std::unique_ptr<Mode>;
    using WeakPtr = std::weak_ptr<Mode>;

    // Configuration for the operation mode
    struct Config {
        rclcpp::Node::SharedPtr node;                                           // ROS 2 node ptr (in case the mode needs to create publishers, subscribers, etc.)
        std::function<State()> get_vehicle_state;                               // Function pointer to get the current state of the vehicle            
        std::function<void(const Eigen::Vector3d &, float)> set_position;       // Function pointer to set the position of the vehicle
        std::function<void(const Eigen::Vector3d &, float)> set_attitude;       // Function pointer to set the attitude of the vehicle
        std::function<void(const Eigen::Vector3d &, float)> set_attitude_rate;  // Function pointer to set the attitude rate of the vehicle
    };

    // Custom constructor like function - as we must have the default constructor for the pluginlib
    inline void initialize_mode(const Mode::Config & config) {

        // Initialize the base class
        node_ = config.node;
        get_vehicle_state = config.get_vehicle_state;
        set_position = config.set_position;
        set_attitude = config.set_attitude;
        set_attitude_rate = config.set_attitude_rate;

        // Initialize the derived class
        initialize();
    }

    // Methods that can be implemented by derived classes
    // that are executed by the state machine when entering, exiting or updating the mode
    virtual void initialize() = 0;
    virtual bool enter() = 0;
    virtual bool exit() = 0;
    virtual void update(double dt) = 0;

protected:

    // The ROS 2 node
    rclcpp::Node::SharedPtr node_{nullptr};

    // Function pointer which will be instantiated with the function pointers passed in the configuration
    std::function<State()> get_vehicle_state{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_position{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_attitude{nullptr};
    std::function<void(const Eigen::Vector3d &, float)> set_attitude_rate{nullptr};
};

} // namespace autopilot