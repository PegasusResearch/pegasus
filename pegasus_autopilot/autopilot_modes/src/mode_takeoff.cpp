#include "autopilot_modes/mode_takeoff.hpp"
#include "pegasus_utils/rotations.hpp"

namespace autopilot {

TakeoffMode::~TakeoffMode() {}

void TakeoffMode::initialize() {

    // Initialize the default target altitude
    node_->declare_parameter<float>("autopilot.TakeoffMode.takeoff_altitude", -1.0f);
    target_altitude = node_->get_parameter("autopilot.TakeoffMode.takeoff_altitude").as_double();

    // Initialize the service server for setting the takeoff altitude
    node_->declare_parameter<std::string>("autopilot.TakeoffMode.set_takeoff_altitude_service", "set_takeoff_altitude");
    altitude_service_ = node_->create_service<pegasus_msgs::srv::Takeoff>(node_->get_parameter("autopilot.TakeoffMode.set_takeoff_altitude_service").as_string(), std::bind(&TakeoffMode::altitude_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Log that the takeoff mode has been initialized successfully
    RCLCPP_INFO(this->node_->get_logger(), "TakeoffMode initialized");
}

bool TakeoffMode::enter() {
    
    // Get the current position and orientation of the drone
    State curr_state = this->get_vehicle_state();

    // Set the target position and attitude to the current position and attitude of the drone
    this->takeoff_pos[0] = curr_state.position[0];
    this->takeoff_pos[1] = curr_state.position[1];
    this->takeoff_pos[2] = curr_state.position[2] + this->target_altitude;

    // TODO: Check if we need to convert the yaw from rad to deg to be used by the target position
    this->takeoff_yaw = Pegasus::Rotations::yaw_from_quaternion(curr_state.attitude);

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool TakeoffMode::exit() {
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void TakeoffMode::update(double) {

    // Set the controller to track the target position and attitude
    this->set_position(this->takeoff_pos, this->takeoff_yaw);
}

void TakeoffMode::altitude_callback(const pegasus_msgs::srv::Takeoff::Request::SharedPtr request, const pegasus_msgs::srv::Takeoff::Response::SharedPtr response) {

    // Set the target altitude
    this->target_altitude = request->height;
    response->success = pegasus_msgs::srv::Takeoff::Response::TRUE;

    RCLCPP_WARN(this->node_->get_logger(), "Takeoff altitude set to %.2f", this->target_altitude);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::TakeoffMode, autopilot::Mode)