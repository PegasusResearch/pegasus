#pragma once

#include <Eigen/Dense>

namespace PegasusAutopilot {

// State of the vehicle
struct State {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};           // Position of the vehicle (FRD) in the world frame NED
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};           // Velocity of the vehicle (FRD) with respect to the world frame NED expressed in the world frame NED
    Eigen::Quaterniond attitude{1.0, 0.0, 0.0, 0.0};             // Attitude of the vehicle (FRD) with respect to the world frame NED expressed in the world frame NED
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};   // Angular velocity of the vehicle (FRD) with respect to the world frame NED expressed in the body frame FRD
};

// Status of the vehicle
struct VehicleStatus {
    bool armed{false};      // Whether the vehicle is armed
    bool flying{false};     // Whether the vehicle is flying
    bool offboard{false};   // Whether the vehicle is in offboard mode
};

}


