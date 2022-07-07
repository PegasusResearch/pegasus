
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
rclcpp_action::GoalResponse MAVLinkNode::takeoff_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pegasus_msgs::action::VehicleTakeOff::Goal> goal) {
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with take_off: " << goal->take_off << "  with height %d: " << goal->height);
    (void)uuid; // To avoid getting warnings about unused parameter during compilation
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief 
 * 
 * @param goal_handle 
 * @return rclcpp_action::CancelResponse 
 */
rclcpp_action::CancelResponse MAVLinkNode::takeoff_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleTakeOff>> goal_handle) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request to cancel goal");
    (void) goal_handle; // To avoid getting warnings about unused parameter during compilation
    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 
 * 
 * @param goal_handle 
 */
void MAVLinkNode::takeoff_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleTakeOff>> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<pegasus_msgs::action::VehicleTakeOff>> goal_handle) {
        
        // Prepare the result message of the action
        std::shared_ptr<pegasus_msgs::action::VehicleTakeOff::Result> result = std::make_shared<pegasus_msgs::action::VehicleTakeOff::Result>();

        // Inform the user that the takeoff action is being performed
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Vehicle is taking off");

        // Set the takeoff altitude to be the one received in the message
        this->action_->set_takeoff_altitude_async(goal_handle->get_goal()->height, [&result, this](mavsdk::Action::Result r){
            (void) r;
            result->success = static_cast<signed char>(this->action_->takeoff());
        });
        

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), result->success);
        goal_handle->succeed(result);

    }, goal_handle}.detach();
}