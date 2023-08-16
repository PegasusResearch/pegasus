#pragma once

#include <autopilot/mode.hpp>
#include "paths/path.hpp"

namespace autopilot {

class FollowTrajectoryMode : public autopilot::Mode {

public:

    ~FollowTrajectoryMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    // The path for the vehicle to follow    
    //Pegasus::Paths::Path path_;
};
}