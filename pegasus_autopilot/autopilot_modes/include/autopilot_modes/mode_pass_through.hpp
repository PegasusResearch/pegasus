#pragma once

#include "mode.hpp"

namespace PegasusAutopilot {

class PassThroughMode : public Mode {

public:

    ~PassThroughMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;
};

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PegasusAutopilot::PassThroughMode, PegasusAutopilot::Mode)