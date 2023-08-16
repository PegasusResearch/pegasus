#pragma once

#include <autopilot/mode.hpp>

namespace autopilot {

class ArmMode : public autopilot::Mode {

public:

    ~ArmMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

};

}