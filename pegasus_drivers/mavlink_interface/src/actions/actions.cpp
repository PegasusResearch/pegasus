#include "rclcpp/rclcpp.hpp"
#include "mavlink_node.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>

/**
 * @brief Method that is used to initialize all the ROS actions (takeoff, land, etc.)
 */
void MAVLinkNode::initializeROSActions() {

    // Initialize the service that enables the vehicle to arm
    this->declare_parameter<std::string>("actions.arm", "arm");
    rclcpp::Parameter arm_topic = this->get_parameter("actions.arm");
    this->arm_disarm_action_server_ = rclcpp_action::create_server<pegasus_msgs::action::VehicleArm>(
        this, 
        arm_topic.as_string(), 
        std::bind(&MAVLinkNode::arm_disarm_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MAVLinkNode::arm_disarm_cancel, this, std::placeholders::_1),
        std::bind(&MAVLinkNode::arm_disarm_accepted, this, std::placeholders::_1));

    // Initialize the service that enables the vehicle to take off
    this->declare_parameter<std::string>("actions.takeoff", "takeoff");
    rclcpp::Parameter takeoff_topic = this->get_parameter("actions.takeoff");
    this->take_off_action_server_ = rclcpp_action::create_server<pegasus_msgs::action::VehicleTakeOff>(
        this, 
        takeoff_topic.as_string(), 
        std::bind(&MAVLinkNode::takeoff_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MAVLinkNode::takeoff_cancel, this, std::placeholders::_1),
        std::bind(&MAVLinkNode::takeoff_accepted, this, std::placeholders::_1));

    // Initialize the service that enables the vehicle to land
    this->declare_parameter<std::string>("actions.land", "land");
    rclcpp::Parameter land_topic = this->get_parameter("actions.land");
    this->land_action_server_ = rclcpp_action::create_server<pegasus_msgs::action::VehicleLand>(
        this, 
        land_topic.as_string(), 
        std::bind(&MAVLinkNode::land_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MAVLinkNode::land_cancel, this, std::placeholders::_1),
        std::bind(&MAVLinkNode::land_accepted, this, std::placeholders::_1));
}