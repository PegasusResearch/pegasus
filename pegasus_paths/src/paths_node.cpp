#include "paths_node.hpp"

/**
 * @brief Construct a new PathsNode object
 * @param node_name The ROS2 node name
 * @param intra_process_comms Whether to use interprocess communication framework or not (false by default)
 */
PathsNode::PathsNode(const std::string & node_name, bool intra_process_comms) : 
    rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {

    
}

/**
 * @brief Destroy the Paths Node object
 */
PathsNode::~PathsNode() {

}

/**
 * @defgroup initFunctions 
 * This group defines all the private initialization functions
 * that will initialize all the ROS publishers, subscribers, services and actions
 */

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 publishers
 */
void PathsNode::init_publishers() {
    
    // ------------------------------------------------------------------------
    // Initialize the publisher for the path points message (used by RVIZ and others)
    // ------------------------------------------------------------------------
    declare_parameter("topics.publishers.points", "path/points");
    points_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("topics.publishers.points").as_string(), 1);

    // ------------------------------------------------------------------------
    // Initialize the publisher for the path data actually used by controllers
    // ------------------------------------------------------------------------
    declare_parameter("topics.publishers.reference", "path/reference");
    path_pub_ = create_publisher<pegasus_msgs::msg::Path>(get_parameter("topics.publishers.reference").as_string(), 1);
}   

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 subscribers
 */
void PathsNode::init_subscribers() {
    
    // ------------------------------------------------------------------------
    // Subscribe to the state of the vehicle (usefull when sending waypoints)
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.subscribers.state", "nav/state");
    state_sub_ = create_subscription<pegasus_msgs::msg::State>(get_parameter("topics.subscribers.state").as_string(), 1, std::bind(&PathsNode::state_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the virtual target state (path parametric value gamma)
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.subscribers.gamma", "nav/gamma");
    gamma_sub_ = create_subscription<std_msgs::msg::Float64>(get_parameter("topics.subscribers.gamma").as_string(), 1, std::bind(&PathsNode::gamma_callback, this, std::placeholders::_1));

}

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 services
 */
void PathsNode::init_services() {

}

/**
 * @defgroup servicesCallbacks
 * This group defines all the service server callbacks,
 * such as arming/disarming the vehicle or auto-landing
 */

/**
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr request, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response) {
    (void) *request; // do nothing with the empty request and avoid compilation warnings from unused argument
    path_.clear();
    response->success = true;
}

/**
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::add_arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, const pegasus_msgs::srv::AddArc::Response::SharedPtr response) {
    
    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Arc object
    auto arc = std::make_shared<Pegasus::Paths::Arc>(speed, Eigen::Vector3d(request->start.data()), Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->clockwise_direction);

    // Add the new arc to the path
    path_.push_back(arc);

    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::add_line_callback(const pegasus_msgs::srv::AddLine::Request::SharedPtr request, const pegasus_msgs::srv::AddLine::Response::SharedPtr response) {
    
    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Line object
    auto line = std::make_shared<Pegasus::Paths::Line>(speed, Eigen::Vector3d(request->start.data()), Eigen::Vector3d(request->end.data()));

    // Add the new line to the path
    path_.push_back(line);

    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::add_circle_callback(const pegasus_msgs::srv::AddCircle::Request::SharedPtr request, const pegasus_msgs::srv::AddCircle::Response::SharedPtr response) {
    
    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Circle object
    auto circle = std::make_shared<Pegasus::Paths::Circle>(speed, Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->radius);

    // Add the new circle to the path
    path_.push_back(circle);

    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}

/**
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::add_lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, const pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response) {

    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Lemniscate object
    auto lemniscate = std::make_shared<Pegasus::Paths::Lemniscate>(speed, Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->radius);

    // Add the new circle to the path
    path_.push_back(lemniscate);

    (void) *response; // do nothing with the empty response and avoid compilation warnings from unused argument
}


/**
 * @defgroup subscriberCallbacks
 * This group defines all the ROS subscriber callbacks
 */

/**
 * @ingroup subscriberCallbacks
 * @brief Callback called by "state_sub_" used to update the current position of the vehicle
 * @param msg A message with the current state of the vehicle
 */
void PathsNode::state_callback(const pegasus_msgs::msg::State::SharedPtr msg) {
    vehicle_pos_[0] = msg->pose.pose.position.x;
    vehicle_pos_[1] = msg->pose.pose.position.y;
    vehicle_pos_[2] = msg->pose.pose.position.z;
}

/**
 * @ingroup subscriberCallbacks
 * @brief Callback called by "gamma_sub_" used to update the current path parametric value (gamma)
 * @param msg A message with a float with the current value of gamma
 */
void PathsNode::gamma_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    gamma_ = msg->data;
}