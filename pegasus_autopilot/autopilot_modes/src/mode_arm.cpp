#include <chrono>
#include "pegasus_utils/rotations.hpp"
#include "autopilot_modes/mode_arm.hpp"

namespace autopilot {

ArmMode::~ArmMode() {}

void ArmMode::initialize() {

    // Create the callback group and executor for the service clients
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Initialize the ROS 2 service clients
    node_->declare_parameter<std::string>("autopilot.ArmMode.arm_service", "arm");
    node_->declare_parameter<std::string>("autopilot.ArmMode.offboard_service", "offboard");

    arm_client_ = node_->create_client<pegasus_msgs::srv::Arm>(node_->get_parameter("autopilot.ArmMode.arm_service").as_string(), rmw_qos_profile_system_default, callback_group_);
    offboard_client_ = node_->create_client<pegasus_msgs::srv::Offboard>(node_->get_parameter("autopilot.ArmMode.offboard_service").as_string(), rmw_qos_profile_system_default, callback_group_);

    // Log that the ArmMode has been initialized successfully 
    RCLCPP_INFO(this->node_->get_logger(), "ArmMode initialized");
}

bool ArmMode::arm() {
    
    // Check if the vehicle is already armed
    if (get_vehicle_status().armed) {
        RCLCPP_WARN(this->node_->get_logger(), "Vehicle is already armed");
        return true;
    } 

    // Arm the vehicle by invoking the service 
    auto arm_request = std::make_shared<pegasus_msgs::srv::Arm::Request>();
    arm_request->arm = true;

    // Wait until the service is available
    while (!arm_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->node_->get_logger(), "service not available, waiting again...");
    }

    // Send the arm request asynchronously
    auto result = arm_client_->async_send_request(arm_request);
    
    // Wait for the result.
    auto timeout = std::chrono::seconds(5);
    if (callback_group_executor_.spin_until_future_complete(result, timeout) != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    // Check if the vehicle was armed successfully
    return result.get()->success == pegasus_msgs::srv::Arm::Response::SUCCESS ? true : false;
}

bool ArmMode::offboard() {
    
    // Check if the vehicle is already in offboard mode
    if (get_vehicle_status().offboard) return true;

    // Set the vehicle to offboard mode by invoking the service 
    auto offboard_request = std::make_shared<pegasus_msgs::srv::Offboard::Request>();

    // Wait until the service is available
    while (!offboard_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->node_->get_logger(), "service not available, waiting again...");
    }

    // Send the offboard request asynchronously
    auto result = offboard_client_->async_send_request(offboard_request);
    
    // Wait for the result.
    auto timeout = std::chrono::seconds(5);
    if (callback_group_executor_.spin_until_future_complete(result, timeout) != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    // Check if the vehicle was set to offboard mode successfully
    return result.get()->success == pegasus_msgs::srv::Offboard::Response::SUCCESS ? true : false;
}

void ArmMode::send_no_thrust_commands() {
    // Get the current state of the vehicle
    State state = get_vehicle_state();

    // Set the target attitude and thrust force to the vehicle
    Eigen::Vector3d target_attitude = Pegasus::Rotations::quaternion_to_euler(state.attitude);

    // Set the target attitude and thrust force to the vehicle (0.4 Newtons of thrust)
    this->set_attitude(target_attitude, 0.4);
}

bool ArmMode::enter() {

    // Attempt to arm the vehicle
    if (!arm()) return false;

    // Send a few offboard commands to put the vehicle in offboard mode automatically
    for (int i = 0; i < 10; i++) {
        send_no_thrust_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Attempt to set the vehicle to offboard mode
    if (!offboard()) return false;

    return true;
}

bool ArmMode::exit() {
    // Do nothing, just return true
    return true;
}

void ArmMode::update(double) {

    // Set the vehicle to spin the motors at zero thrust
    send_no_thrust_commands();
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::ArmMode, autopilot::Mode)