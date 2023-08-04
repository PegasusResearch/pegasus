#pragma once

#include "mode.hpp"

namespace Pegasus {

class TakeoffMode : public Mode {

public:

    TakeoffMode(const Mode::Config & config) : Mode(config) {}
    ~TakeoffMode() {}

    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The target altitude to takeoff to
    float target_altitude{1.0f};

    // The position the vehicle is taking off from
    Eigen::Vector3d takeoff_pos{Eigen::Vector3d::Zero()};
    float takeoff_yaw{0.0f};
};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Pegasus::TakeoffMode, Pegasus::Mode)