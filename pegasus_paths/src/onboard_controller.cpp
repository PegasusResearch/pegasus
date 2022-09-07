#include "onboard_controller.hpp"

/**
 * @brief Construct a new Onboard controller
 * @param nh The nodehandler shared pointer for the base class that creates the controller object
 * @param path A shared pointer for a path the controller must track
 * @param controller_rate The rate at which the controller will operate
 */
OnboardController::OnboardController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate) : BaseControllerNode(nh, path, controller_rate) {
    // TODO
}

/**
 * @brief Destroy the Onboard controller object
 */
OnboardController::~OnboardController() {
    // TODO
}

/**
 * @brief Method to start the path following controller
 */
void OnboardController::start() {
    // TODO
}

/**
 * @brief Method for stoping the path following controller
 */
void OnboardController::stop() {
    // TODO
}

    /**
 * @brief Method that is called whenever the reference path to follow object is reset. This method should
 * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
 */
void OnboardController::reset() {
    // TODO
}

/**
 * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
 * "current_velocity_".
 * @param msg A message with the state of the vehicle
 */
void OnboardController::update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg) {
    (void) msg;
    // TODO
}

/**
 * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
 * which is used to update the control signals
 */
void OnboardController::controller_update() {
    // TODO
}