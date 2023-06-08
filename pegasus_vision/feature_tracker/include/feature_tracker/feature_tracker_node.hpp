#pragma once

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>

namespace Pegasus {

class FeatureTrackerNode : public rclcpp::Node {

public:

    /**
     * @brief An alias for a shared, unique and weak pointers of a FeatureTrackerNode
     */
    using SharedPtr = std::shared_ptr<FeatureTrackerNode>;
    using UniquePtr = std::unique_ptr<FeatureTrackerNode>;
    using WeakPtr = std::weak_ptr<FeatureTrackerNode>;

    /**
     * @brief An alias for the image sync policy to make sure that the rgb and depth images are synchronized 
     */
    using ApproximateTime = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    /**
     * @brief Default constructor
     */
    FeatureTrackerNode();

protected:

    /**
     * @brief Initialize the class subscribers
     */
    void initialize_subscribers();

    /**
     * @brief Initialize the class publishers
     */
    void initialize_publishers();

    /**
     * @brief Callback for the image topic
     * @param msg The image message
     */
    void rgb_depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

    /**
     * @brief Callback for the imu topic
     * @param msg The imu message
     */
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
    const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;


    std::shared_ptr<message_filters::Synchronizer<ApproximateTime>> image_synchronizer_;
};

}