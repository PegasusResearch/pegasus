#pragma once

#include <autopilot/mode.hpp>
#include <Eigen/Core>

namespace autopilot {

class HoldMode : public autopilot::Mode {

public:

    ~HoldMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The target position and attitude for the vehicle to hold to
    Eigen::Vector3d target_pos{Eigen::Vector3d::Zero()};
    float target_yaw{0.0f};
};

}