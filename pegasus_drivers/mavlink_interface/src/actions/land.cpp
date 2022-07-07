
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "mavlink_node.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>

/**************************************
 * ROS2 Actions
 **************************************/

/**
 * @brief TODO
 * 
 * @param uuid 
 * @param goal 
 * @return rclcpp_action::GoalResponse 
 */
rclcpp_action::GoalResponse MAVLinkNode::land_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pegasus_msgs::action::VehicleLand::Goal> goal) {
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with land: " << goal->land);
    (void)uuid; // To avoid getting warnings about unused parameter during compilation
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief 
 * 
 * @param goal_handle 
 * @return rclcpp_action::CancelResponse 
 */
rclcpp_action::CancelResponse MAVLinkNode::land_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleLand>> goal_handle) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request to cancel goal");
    (void) goal_handle; // To avoid getting warnings about unused parameter during compilation
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 
 * 
 * @param goal_handle 
 */
void MAVLinkNode::land_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleLand>> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleLand>> goal_handle) {
        
        // Prepare the result message of the action
        std::shared_ptr<pegasus_msgs::action::VehicleLand::Result> result = std::make_shared<pegasus_msgs::action::VehicleLand::Result>();

        // Inform the user that the takeoff action is being performed
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Vehicle is landing");
        result->success = static_cast<signed char>(this->action_->land());

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), result->success);
        goal_handle->succeed(result);

    }, goal_handle}.detach();
}