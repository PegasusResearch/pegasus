#include "feature_tracker_node.hpp"

namespace Pegasus {

/**
 * @brief Default constructor
 */
FeatureTrackerNode::FeatureTrackerNode() : Node("feature_tracker_node") {

    // Initialize the publishers
    initialize_publishers();

    // Initialize the subscribers
    initialize_subscribers();
}

/**
 * @brief Initialize the class subscribers
 */
void FeatureTrackerNode::initialize_subscribers() {
    
    // ------------------------------------------------------------------------
    // Initialize the subscriber for the image topic
    // ------------------------------------------------------------------------
    declare_parameter("subscribers.rgb_camera", "camera/rgb");
    declare_parameter("subscribers.depth_camera", "camera/depth");
    
    // Subscribe to the rgb and depth images
    rgb_image_sub_.subscribe(this->shared_from_this(), get_parameter("subscribers.rgb_camera").as_string(), rmw_qos_profile_sensor_data);
    depth_image_sub_.subscribe(this->shared_from_this(), get_parameter("subscribers.depth_camera").as_string(), rmw_qos_profile_sensor_data);

    // Create the image synchronizer
    image_synchronizer_ = std::make_shared<message_filters::Synchronizer<ApproximateTime>>(ApproximateTime(1), rgb_image_sub_, depth_image_sub_);
    image_synchronizer_->registerCallback(std::bind(&FeatureTrackerNode::rgb_depth_image_callback, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Initialize the class publishers
 */
void FeatureTrackerNode::initialize_publishers() {

    // ------------------------------------------------------------------------
    // Initialize the publisher for the pointcloud topic
    // ------------------------------------------------------------------------
    // declare_parameter("publishers.pointcloud", "pointcloud");
    // rclcpp::Parameter pointcloud_topic = get_parameter("publishers.pointcloud");
    // RCLCPP_INFO(get_logger(), "Publishing pointcloud to topic: %s", pointcloud_topic.as_string().c_str());

}

/**
 * @brief Callback for the image topic
 * @param msg The image message
 */
// void FeatureTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     (void) msg;
//     return;
// }

/**
 * @brief Callback for the image topic
 * @param msg The image message
 */
void FeatureTrackerNode::rgb_depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg) {
    (void) rgb_msg;
    (void) depth_msg;
    return; 
}

}

/**
 * The entry point of the feature tracker node
 */
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pegasus::FeatureTrackerNode>());
    rclcpp::shutdown();
    return 0;
}