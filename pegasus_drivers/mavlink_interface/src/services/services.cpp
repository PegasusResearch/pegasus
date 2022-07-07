#include "mavlink_node.hpp"

/**
 * @brief Method that is used to initialize all the ROS services (emergency kill switch)
 */
void MAVLinkNode::initializeROSSevices() {

    // Initialize the emergency kill switch service
    this->declare_parameter<std::string>("services.kill_switch", "kill");
    rclcpp::Parameter kill_topic = this->get_parameter("services.kill_switch");
    this->kill_switch_service_ = this->create_service<pegasus_msgs::srv::KillSwitch>(
        kill_topic.as_string(), std::bind(&MAVLinkNode::kill_switch, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief TODO - Method that is called when a kill switch is requested
 * 
 * @param request A boolean with a request to enable a kill switch
 * @param response A boolean whether the kill switch as requested asynchronously
 */
void MAVLinkNode::kill_switch(const std::shared_ptr<pegasus_msgs::srv::KillSwitch::Request> request, const std::shared_ptr<pegasus_msgs::srv::KillSwitch::Response> response) {

    // If the request boolean is false, just return true (nothing is done and return)
    if(!request->kill) {
        response->result = true;
        return;
    }

    // Define an anonymous function for publishing the result
    this->action_->kill_async([](mavsdk::Action::Result result) {
        if(result == mavsdk::Action::Result::Success) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Vehicle kill-switch activated.");
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Failed to activate vehicle kill-switch. Error: " << result);
        }   
    });
}