#pragma once

#include "mode.hpp"

namespace Pegasus {

class PassTroughMode : public Mode {

public:

    PassTroughMode(const Mode::Config & config) : Mode(config) {}
    ~PassTroughMode() {}

    bool enter() override;
    bool exit() override;
    void update(double dt) override;
};

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Pegasus::PassTroughMode, Pegasus::Mode)