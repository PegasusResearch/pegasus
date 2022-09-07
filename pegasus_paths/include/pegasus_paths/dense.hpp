#pragma once

// Generic base class to define a ROS2 path following/Tracjectory Tracking controller
#include "base_controller.hpp"
// A simple position PID controller defined in the inertial frame of reference
#include "pid_controller.hpp"
// A simple feed-forward of a position reference to the vehicle's onboard controller
#include "onboard_controller.hpp"