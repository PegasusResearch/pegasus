#include "paths_node.hpp"

/**
 * @brief Construct a new PathsNode object
 * @param node_name The ROS2 node name
 * @param intra_process_comms Whether to use interprocess communication framework or not (false by default)
 */
PathsNode::PathsNode(const std::string & node_name, bool intra_process_comms) : 
    rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {

    // Read the sample step for obtaining the points that describe the path from the parameter server
    declare_parameter<double>("path.sample_step", 0.0001);
    sample_step_ = get_parameter("path.sample_step").as_double();

    // Read the node_rate at which to run the controllers
    declare_parameter<double>("controllers.rate", 20.0);
    control_rate_ = get_parameter("controllers.rate").as_double();

    // Initialize an empty Path object
    path_ = std::make_shared<Pegasus::Paths::Path>();

    // Initialize the publishers, subscribers and services
    init_publishers();
    init_subscribers();
    init_services();

    RCLCPP_INFO_STREAM(get_logger(), "Paths node is initialized");
}

/**
 * @brief Destroy the Paths Node object
 */
PathsNode::~PathsNode() {
    RCLCPP_INFO_STREAM(get_logger(), "Paths node is destroyed");
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
    declare_parameter("path.topics.publishers.points", "path/points");
    points_pub_ = create_publisher<nav_msgs::msg::Path>(get_parameter("path.topics.publishers.points").as_string(), 1);

    RCLCPP_INFO_STREAM(get_logger(), "Paths node publishers initialized");
}   

/**
 * @ingroup initFunctions
 * @brief Method used to initialize all the ROS2 subscribers
 */
void PathsNode::init_subscribers() {

    // ------------------------------------------------------------------------
    // Initialize the subscriber for the status of the vehicle (check if it is armed and landed)
    // ------------------------------------------------------------------------
    declare_parameter("path.topics.subscribers.status", "status");
    status_sub_ = create_subscription<pegasus_msgs::msg::Status>(get_parameter("path.topics.subscribers.status").as_string(), 1, std::bind(&PathsNode::status_callback, this, std::placeholders::_1));

    // ------------------------------------------------------------------------
    // Subscribe to the state of the vehicle (usefull when sending waypoints)
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("path.topics.subscribers.state", "nav/state");
    state_sub_ = create_subscription<pegasus_msgs::msg::State>(get_parameter("path.topics.subscribers.state").as_string(), 1, std::bind(&PathsNode::state_callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(get_logger(), "Paths node subscribers initialized");
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

    // ------------- MISSION SERVICES --------------

    // ------------------------------------------------------------------------
    // Initiate the service to start a path following/tracking mission
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.start_mission", "path/start_mission");
    start_mission_service_ = create_service<pegasus_msgs::srv::StartMission>(get_parameter("topics.services.start_mission").as_string(), std::bind(&PathsNode::start_mission_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ------------------------------------------------------------------------
    // Initiate the service to start a landing mission
    // ------------------------------------------------------------------------
    declare_parameter<std::string>("topics.services.land_mission", "path/land_mission");
    land_mission_service_ = create_service<pegasus_msgs::srv::LandMission>(get_parameter("topics.services.land_mission").as_string(), std::bind(&PathsNode::land_mission_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Declare the parameter for arming the vehicle service client
    declare_parameter<std::string>("topics.services.arm", "arm");

    // Declare the parameter for auto-landing the vehicle service client
    declare_parameter<std::string>("topics.services.onboard_land", "land");

    RCLCPP_INFO_STREAM(get_logger(), "Paths node services initialized");
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
    
    // Clear the path
    path_->clear();

    // Call the controller reset function if one is running to signal that the path was cleaned
    if(controller_) controller_->reset();

    // Reset the path points message
    path_points_msg_.header.frame_id = "world_ned";
    path_points_msg_.header.stamp = get_clock()->now();

    // Clear the points message vector (such that RVIZ shows a clear path)
    path_points_msg_.poses.clear();
    points_pub_->publish(path_points_msg_);

    // Make the response of this service to true
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Path reset");
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
    auto arc = std::make_shared<Pegasus::Paths::Arc>(speed, Eigen::Vector2d(request->start.data()), Eigen::Vector3d(request->center.data()), Eigen::Vector3d(request->normal.data()), request->clockwise_direction);

    // Add the new arc to the path
    add_section_to_path(arc);

    // Update the response
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Add arc to path");
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
    add_section_to_path(line);

    // Update the response
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Add line to path");
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
    add_section_to_path(circle);

    // Update the response
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Add circle to path");
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
    add_section_to_path(lemniscate);

    // Update the response
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Add lemniscate to path");
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
    auto last_path_pos = path_->get_last_pd();

    // If the path is empty, create a waypoint between the current vehicle position and the desired waypoint
    if(!last_path_pos.has_value()) {
        line = std::make_shared<Pegasus::Paths::Line>(speed, vehicle_pos_, Eigen::Vector3d(request->end.data()));
    // If the path is NOT empty, create a waypoint between the path last position and the desired waypoint
    } else {
        line = std::make_shared<Pegasus::Paths::Line>(speed, last_path_pos.value(), Eigen::Vector3d(request->end.data()));
    }

    // Add the new line to the path
    add_section_to_path(line);

    // Update the response
    response->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "Add waypoint to path");
}

/**
 * @brief Auxiliar method that should be called by the services to add a new section to the path
 * @param section A shared pointer to a path section
 */
void PathsNode::add_section_to_path(const Pegasus::Paths::Section::SharedPtr section) {
    
    // Add the new section to the path
    path_->push_back(section);

    // Update the samples of the path
    auto path_samples = path_->get_samples(sample_step_);

    // If the path samples optional is not null, update the message that describes the path
    if(path_samples.has_value()) {

        // Set the header of the message
        path_points_msg_.header.frame_id = "world_ned";
        path_points_msg_.header.stamp = get_clock()->now();

        // Clear the points message vector
        path_points_msg_.poses.clear();

        // Create a new vector with new samples
        for (auto & sample: path_samples.value()) {
            
            // Create a pose message object
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = sample[0];
            pose.pose.position.y = sample[1];
            pose.pose.position.z = sample[2];

            // Add it to the list of points message
            path_points_msg_.poses.push_back(pose);
        }   

        // Publish the points message
        points_pub_->publish(path_points_msg_);

        RCLCPP_INFO_STREAM(get_logger(), "Sampled the Path");
    }
}

/**
 * @ingroup servicesCallbacks
 * @brief TODO
 * @param request 
 * @param response 
 */
void PathsNode::start_mission_callback(const pegasus_msgs::srv::StartMission::Request::SharedPtr request, const pegasus_msgs::srv::StartMission::Response::SharedPtr response) {
    
    // TODO - improve this section and implement a FACTORY PATTERN similar to the THRUST CURVE package
    // TODO - improve the swaping of controllers to make sure the drone does not fall when hot-swapping
    RCLCPP_INFO_STREAM(get_logger(), "START MISSION CALLBACK");

    // If the controller is null || the controller instance is the same as the already requested controller, 
    // then do not create a new instance. Just check if the drone is armed and carry on
    if (!(controller_ != nullptr && controller_->get_identifier().compare(request->controller_name) == 0)) {

        // Check if the required controller to use is a PID controller
        if(request->controller_name.compare("pid") == 0) {
            
            try {
                controller_ = std::make_shared<PidController>(shared_from_this(), path_, control_rate_);
            } catch (std::runtime_error &error) {
                RCLCPP_ERROR_STREAM(get_logger(), error.what());
                return;
            }

            RCLCPP_INFO_STREAM(get_logger(), "Created a PID position tracking controller");
        
        // Check if the required controller to use is the actual onboard controller
        } else if (request->controller_name.compare("onboard") == 0) {
            
            try {
                controller_ = std::make_shared<OnboardController>(shared_from_this(), path_, control_rate_);
            } catch (std::runtime_error &error) {
                RCLCPP_ERROR_STREAM(get_logger(), error.what());
                return;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Using the onboard position tracking controller");
        }
    }

    // Check if the vehicle is un-armed and arm
    if(!armed_) {
        // Create the service client to arm the vehicle
        rclcpp::Client<pegasus_msgs::srv::Arm>::SharedPtr arm_service_client = create_client<pegasus_msgs::srv::Arm>(get_parameter("topics.services.arm").as_string());
        
        // Create the request message
        auto request = std::make_shared<pegasus_msgs::srv::Arm::Request>();
        request->arm = true;

        // Send the arm request
        RCLCPP_INFO_STREAM(get_logger(), "Requesting the vehicle to arm");
        arm_service_client->async_send_request(request);
    }

    // Start the controller
    RCLCPP_INFO_STREAM(get_logger(), "Starting the mission");
    controller_->start();

    // Do nothing with the response object (and avoid compilation warnings)
    (void) response;
}


/**
 * @ingroup servicesCallbacks
 * @brief Service callback responsible for taking action when a landing mission is requested. By default it will always use the onboard
 * controller for this type of mission
 * @param request A pointer to the request object (unused)
 * @param response A pointer to the response object. Contains whether the landing mission will be executed or not, and a string which might contain
 * or not some information regarding the landing of the vehicle
 */
void PathsNode::land_mission_callback(const pegasus_msgs::srv::LandMission::Request::SharedPtr request, const pegasus_msgs::srv::LandMission::Response::SharedPtr response) {

    (void) request; // DO NOTHING, IGNORE THE EMPTY REQUEST OBJECT

    // If the drone is not flying (aka, landed - do nothing and ignore the service)
    if(!flying_) {
        response->auto_landing = false;
        response->comments = std::string("Vehicle already landed");
        return; 
    }

    // Create the service client to arm the vehicle
    rclcpp::Client<pegasus_msgs::srv::Land>::SharedPtr land_service_client = create_client<pegasus_msgs::srv::Land>(get_parameter("topics.services.onboard_land").as_string());

    // Create the request message
    auto autoland_request = std::make_shared<pegasus_msgs::srv::Land::Request>();

    // Wait 2s for the landind service to be available
    if(!land_service_client->wait_for_service(std::chrono::seconds(2))) {
        response->auto_landing = false;
        response->comments = std::string("Autolanding system not available. Request ignored");
        RCLCPP_ERROR_STREAM(get_logger(), "Autolanding system not available. Request ignored");
        return;
    }

    // TODO - improve the safety of this part

    // Stop the current controller, it it is running
    if(controller_ != nullptr) controller_->stop();

    // Send the landing request
    land_service_client->async_send_request(autoland_request);

    response->auto_landing = true;
    response->comments = std::string("Vehicle auto-landing requested");
    RCLCPP_WARN_STREAM(get_logger(), "Vehicle auto-landing requested\n");
}


/**
 * @defgroup subscribersCallbacks
 * This group defines all the callbacks used by the ROS2 subscribers
 */

/**
 * @brief Method that is called by "status_sub_" and updates the variables "armed_" and "flying_"
 * @param msg A pegasus status message
 */
void PathsNode::status_callback(const pegasus_msgs::msg::Status::SharedPtr msg) {

    // Update the armed state of the vehicle
    armed_ = msg->armed;

    // Update the flying state of the vehicle - TODO - get this from the driver
    flying_ = (msg->landed_state == msg->IN_AIR || msg->landed_state == msg->TAKING_OFF) ? true : false;
}

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