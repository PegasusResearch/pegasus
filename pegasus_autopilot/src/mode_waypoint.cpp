#include "mode_waypoint.hpp"

namespace Pegasus {

WaypointMode::WaypointMode(const Mode::Config & config) : Mode(config) {

    // Create the waypoint service server
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(
        "set_waypoint", 
        std::bind(&WaypointMode::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
}

WaypointMode::~WaypointMode() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

bool WaypointMode::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->waypoint_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Waypoint not set - cannot enter waypoint mode");
        return false;
    }

    // Reset the waypoint flag (to make sure we do not enter twice in this mode without setting a new waypoint)
    this->waypoint_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool WaypointMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void WaypointMode::update(double) {

    // Set the controller to track the target position and attitude
    this->set_position(this->target_pos, this->target_yaw);
}

void WaypointMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->target_pos[0] = request->position[0];
    this->target_pos[1] = request->position[1];
    this->target_pos[2] = request->position[2];
    this->target_yaw = request->yaw;

    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
}

}