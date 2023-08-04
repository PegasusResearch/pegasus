#include "mode_takeoff.hpp"


namespace Pegasus {

bool TakeoffMode::enter() {
    
    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->takeoff_pos[0] = curr_state.position[0];
    this->takeoff_pos[1] = curr_state.position[1];
    this->takeoff_pos[2] = curr_state.position[2];
    this->takeoff_yaw = curr_state.attitude.yaw();

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool TakeoffMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void TakeoffMode::update(double) {

    // Set the controller to track the target position and attitude
    this->set_position(this->target_pos, this->target_yaw);
}

}