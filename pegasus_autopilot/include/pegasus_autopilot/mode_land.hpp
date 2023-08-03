#pragma once

#include "mode.hpp"

namespace Pegasus {

class LandMode : public Mode {

public:

    LandMode(const Mode::Config & config);
    ~LandMode();

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