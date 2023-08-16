#pragma once

#include <autopilot/mode.hpp>

namespace autopilot {

class TakeoffMode : public autopilot::Mode {

public:

    ~TakeoffMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The target altitude to takeoff to
    float target_altitude{1.0f};

    // The target position for the vehicle to take off to
    Eigen::Vector3d takeoff_pos{Eigen::Vector3d::Zero()};
    float takeoff_yaw{0.0f};
};

}