#pragma once

#include "mode.hpp"
#include "paths/path.hpp"

namespace Pegasus {

class FollowTrajectoryMode : public Mode {

public:

    FollowTrajectoryMode(const Mode::Config & config);
    ~FollowTrajectoryMode();

    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    // The path for the vehicle to follow    
    Paths::Path path_;
};
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Pegasus::FollowTrajectoryMode, Pegasus::Mode)