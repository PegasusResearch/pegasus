#pragma once

#include "mode.hpp"

namespace Pegasus {

class FollowTrajectoryMode : public Mode {

public:

    FollowTrajectoryMode(const Mode::Config & config);
    ~FollowTrajectoryMode();

    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:


private:

};
}