#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include <memory>

namespace Pegasus {

class FeatureTrackerNode : public rclcpp::Node {

public:

    /**
     * @brief An alias for a shared pointer of a FeatureTrackerNode
     */
    using SharedPtr = std::shared_ptr<FeatureTrackerNode>;

    /**
     * @brief An alias for a unique pointer of a FeatureTrackerNode
     */
    using UniquePtr = std::unique_ptr<FeatureTrackerNode>;

    /**
     * @brief An alias for a weak pointer of a FeatureTrackerNode
     */
    using WeakPtr = std::weak_ptr<FeatureTrackerNode>;

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
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Subscriber for the image topic
     */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    /**
     * @brief Publisher for the pointcloud topic
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;

};

}