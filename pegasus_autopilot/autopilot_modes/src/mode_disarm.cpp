#include "autopilot_modes/mode_disarm.hpp"

namespace autopilot {

DisarmMode::~DisarmMode() {}

void DisarmMode::initialize() {

    // Initialize the ROS 2 service clients
    node_->declare_parameter<std::string>("autopilot.DisarmMode.disarm_service", "disarm");

    // ----- THIS CODE WILL REPLACE THE CODE BELLOW WHEN WE SWITCH TO ROS HUMBLE IN THE VEHICLES ------
    // // Create the callback group and executor for the service clients
    // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface(), false);
    // disarm_client_ = node_->create_client<pegasus_msgs::srv::KillSwitch>(node_->get_parameter("autopilot.DisarmMode.disarm_service").as_string(), rmw_qos_profile_system_default, callback_group_);
    
    // ----- THIS CODE IS ONLY USED IN ROS FOXY --------
    std::string sub_node_name = "autopilot_disarm_client";
    auto options = rclcpp::NodeOptions()
        .start_parameter_services(false)
        .start_parameter_event_publisher(false)
        .arguments({"--ros-args", "-r", "__node:=" + sub_node_name, "--"});
    sub_node_ = rclcpp::Node::make_shared("_", options);
    disarm_client_ = sub_node_->create_client<pegasus_msgs::srv::KillSwitch>(node_->get_parameter("autopilot.DisarmMode.disarm_service").as_string(), rmw_qos_profile_system_default);

    // Log that the DisarmMode has been initialized successfully
    RCLCPP_INFO(this->node_->get_logger(), "DisarmMode initialized");
}

bool DisarmMode::disarm() {
    
    // Check if the vehicle is already armed
    if (!get_vehicle_status().armed) {
        return true;
    } 

    // Arm the vehicle by invoking the service 
    auto arm_request = std::make_shared<pegasus_msgs::srv::KillSwitch::Request>();
    arm_request->kill = true;

    // Wait until the service is available
    while (!disarm_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->node_->get_logger(), "service not available, waiting again...");
    }

    // Send the arm request asynchronously
    auto result = disarm_client_->async_send_request(arm_request);
    
    // Wait for the result.
    auto timeout = std::chrono::seconds(5);
    //if (callback_group_executor_.spin_until_future_complete(result, timeout) != rclcpp::FutureReturnCode::SUCCESS) return false;
    if(rclcpp::spin_until_future_complete(sub_node_, result, timeout) != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    // Check if the vehicle was disarmed successfully
    return result.get()->success == pegasus_msgs::srv::KillSwitch::Response::SUCCESS ? true : false;
}

bool DisarmMode::enter() {

    // Attempt to disarm the vehicle
    disarm();

    // Fow now, just return true, but improve this later on
    return true;
}

bool DisarmMode::exit() {
    // Do nothing, just return true
    return true;
}

void DisarmMode::update(double) {
    // Do nothing, just idle
    return;
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::DisarmMode, autopilot::Mode)