#include "autopilot_modes/mode_pass_through.hpp"

namespace PegasusAutopilot {

PassThroughMode::~PassThroughMode() {}

void PassThroughMode::initialize() {
    // Do nothing
    return;
}

bool PassThroughMode::enter() {
    // TODO: Implement this method
    return true;
}

bool PassThroughMode::exit() {
    // TODO: Implement this method
    return true;
}

void PassThroughMode::update(double) {
    // TODO: Implement this method
    return;
}

}