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

    bool check_land_complete(double curr_z, double prev_z, double dt);

    // The desired landing speed
    double land_speed_;   // [m/s]
    double land_detected_treshold_;    // [m]

    // The next target position for the landing controller to track
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    float target_yaw_{0.0f};

    // The previous position of the drone on the z-axis
    float prev_z_pos_{-100000000.0f};
};

}