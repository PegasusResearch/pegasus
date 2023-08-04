#pragma once

#include "mode.hpp"

namespace PegasusAutopilot {

class DisarmMode : public Mode {

public:

    ~DisarmMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PegasusAutopilot::DisarmMode, PegasusAutopilot::Mode)