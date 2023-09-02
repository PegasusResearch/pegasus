#pragma once

#include <autopilot/mode.hpp>

namespace autopilot {

class PassThroughMode : public autopilot::Mode {

public:

    ~PassThroughMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;
};

}