#pragma once

#include <autopilot/mode.hpp>

namespace autopilot {

class LandMode : public autopilot::Mode {

public:

    ~LandMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

private:

    // Check if the drone has landed
    bool check_land_complete(double velocity_z, double dt);

    // The desired landing speed
    double land_speed_;   // [m/s]
    double land_detected_treshold_;    // [m]
    double countdown_to_disarm_; // [s]

    // Iteration counter for detecting that the vehicle has landed
    double land_counter_;

    // The next target position for the landing controller to track
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    float target_yaw_{0.0f};
};

}