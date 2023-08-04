#pragma once

#include "mode.hpp"

namespace PegasusAutopilot {

class LandMode : public Mode {

public:

    ~LandMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

private:

    // The desired landing speed
    float land_speed_{0.5f};   // [m/s]

    // The next target position for the landing controller to track
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    float target_yaw_{0.0f};
};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PegasusAutopilot::LandMode, PegasusAutopilot::Mode)