#pragma once

#include "mode.hpp"
#include "paths/path.hpp"

namespace PegasusAutopilot {

class FollowTrajectoryMode : public Mode {

public:

    ~FollowTrajectoryMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    // The path for the vehicle to follow    
    Pegasus::Paths::Path path_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PegasusAutopilot::FollowTrajectoryMode, PegasusAutopilot::Mode)