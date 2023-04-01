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
    declare_parameter("publishers.image", "image");
    rclcpp::Parameter image_topic = get_parameter("publishers.status");
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic.as_string(), 
        rclcpp::SensorDataQoS(), 
        std::bind(&FeatureTrackerNode::image_callback, this, std::placeholders::_1)
    );

}

/**
 * @brief Initialize the class publishers
 */
void FeatureTrackerNode::initialize_publishers() {

    // ------------------------------------------------------------------------
    // Initialize the publisher for the pointcloud topic
    // ------------------------------------------------------------------------
    declare_parameter("publishers.pointcloud", "pointcloud");
    rclcpp::Parameter pointcloud_topic = get_parameter("publishers.pointcloud");
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        pointcloud_topic.as_string(), 
        rclcpp::SensorDataQoS()
    );

}

/**
 * @brief Callback for the image topic
 * @param msg The image message
 */
void FeatureTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    (void) msg;
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