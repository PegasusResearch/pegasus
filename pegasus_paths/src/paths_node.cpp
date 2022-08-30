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
 * @defgroup state_machine_callbacks
 * This section defines all the callbacks that are responsible for transitions in the node state machine
 */

/**
 * @ingroup state_machine_callbacks
 * @brief on_activate callback is being called when the lifecycle node
 * enters the "activating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "active" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PathsNode::on_activate(const rclcpp_lifecycle::State & state) {
    
    // Call the default on_activate method
    LifecycleNode::on_activate(state);

    // Initialize the publishers, subscribers and services
    init_publishers();
    init_subscribers();
    init_services();

    // Return success
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @ingroup state_machine_callbacks
 * @brief on_deactivate callback is being called when the lifecycle node
 * enters the "deactivating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "active".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "active"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PathsNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    
    // Call the default on_activate method
    LifecycleNode::on_deactivate(state);

    // Stop the publishers
    points_pub_.reset();
    path_pub_.reset();

    // Stop the subscribers
    state_sub_.reset();
    gamma_sub_.reset();

    // Stop the services
    reset_service_.reset();
    add_arc_service_.reset();
    add_line_service_.reset();
    add_circle_service_.reset();
    add_lemniscate_service_.reset();

    // Return success
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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
    
    // ------------------------------------------------------------------------
    // Initiate the service to reset the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.reset", "path/reset");
    reset_service_ = create_service<pegasus_msgs::srv::ResetPath>(get_parameter("topics.services.reset").as_string(), std::bind(&PathsNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to add an arc to the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.arc", "path/add_arc");
    add_arc_service_ = create_service<pegasus_msgs::srv::AddArc>(get_parameter("topics.services.arc").as_string(), std::bind(&PathsNode::add_arc_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to add a line to the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.line", "path/add_line");
    add_line_service_ = create_service<pegasus_msgs::srv::AddLine>(get_parameter("topics.services.line").as_string(), std::bind(&PathsNode::add_line_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to add a circle to the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.circle", "path/add_circle");
    add_circle_service_ = create_service<pegasus_msgs::srv::AddCircle>(get_parameter("topics.services.circle").as_string(), std::bind(&PathsNode::add_circle_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to add a circle to the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.lemniscate", "path/add_lemniscate");
    add_lemniscate_service_ = create_service<pegasus_msgs::srv::AddLemniscate>(get_parameter("topics.services.lemniscate").as_string(), std::bind(&PathsNode::add_lemniscate_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to add a waypoint to the path
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.waypoint", "path/add_waypoint");
    add_waypoint_service_ = create_service<pegasus_msgs::srv::AddWaypoint>(get_parameter("topics.services.waypoint").as_string(), std::bind(&PathsNode::add_waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
 */
void PathsNode::timer_callback() {

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

    // Update the response
    response->success = true;
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

    // Update the response
    response->success = true;
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

    // Update the response
    response->success = true;
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

    // Update the response
    response->success = true;
}

/**
 * @ingroup servicesCallbacks
 * @brief TODO
 * @param request
 * @param response
 */
void PathsNode::add_waypoint_callback(const pegasus_msgs::srv::AddWaypoint::Request::SharedPtr request, const pegasus_msgs::srv::AddWaypoint::Response::SharedPtr response) {

    // Create a new Speed object (by default just get the value for now - TO BE IMPROVED)
    auto speed = std::make_shared<Pegasus::Paths::ConstSpeed>(request->speed.parameters[0]);

    // Create a new Line object
    Pegasus::Paths::Line::SharedPtr line;

    // Try to get the last position of the current path
    auto last_path_pos = path_.get_last_pd();

    // If the path is empty, create a waypoint between the current vehicle position and the desired waypoint
    if(!last_path_pos.has_value()) {
        line = std::make_shared<Pegasus::Paths::Line>(speed, vehicle_pos_, Eigen::Vector3d(request->end.data()));
    // If the path is NOT empty, create a waypoint between the path last position and the desired waypoint
    } else {
        line = std::make_shared<Pegasus::Paths::Line>(speed, last_path_pos.value(), Eigen::Vector3d(request->end.data()));
    }

    // Add the new line to the path
    path_.push_back(line);

    // Update the response
    response->success = true;
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

    // Update the current path parametric value
    gamma_ = (double) msg->data;

    // Evaluate the path at the given gamma
    auto data = path_.get_all_data(gamma_);

    // If the data is not a null optional, then publish the message with all the data
    if(data.has_value()) {

       auto data_value = data.value();

        // Get the position, first and second derivatives
        for(unsigned int i = 0; i < 3; i++) {
            path_msg_.pd[i] = data_value.pd[i];
            path_msg_.d_pd[i] = data_value.d_pd[i];
            path_msg_.dd_pd[i] = data_value.dd_pd[i];
        }

        // Get other path statistics
        path_msg_.curvature = data_value.curvature;
        path_msg_.torsion = data_value.torsion;
        path_msg_.tangent_angle = data_value.tangent_angle;
        path_msg_.derivative_norm = data_value.derivative_norm;

        // Get the desired speed assignments
        path_msg_.vehicle_speed = data_value.vehicle_speed;
        path_msg_.vd = data_value.vd;
        path_msg_.d_vd = data_value.d_vd;

        // Get the bounds of the path parametric values
        path_msg_.gamma_min = data_value.min_gamma;
        path_msg_.gamma_max = data_value.max_gamma;

        // The parametric value at which this evaluation was made
        path_msg_.gamma = gamma_;

        // For now, let the desired yaw be the tangent to the path
        path_msg_.yaw = data_value.tangent_angle;

        // Publish the path information
        path_pub_->publish(path_msg_);
    }
}