#pragma once

#include "mode.hpp"
#include <Eigen/Core>

namespace Pegasus {

class HoldMode : public Mode {

public:

    HoldMode(const Mode::Config & config) : Mode(config) {}
    ~HoldMode() {}

    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The target position and attitude for the vehicle to hold to
    Eigen::Vector3d target_pos{Eigen::Vector3d::Zero()};
    float target_yaw{0.0f};
};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Pegasus::HoldMode, Pegasus::Mode)