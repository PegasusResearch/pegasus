#pragma once

#include "paths/dense.hpp"
#include "rclcpp/rclcpp.hpp"

// Messages for the status of the vehicle/ and other driver services
#include "pegasus_msgs/srv/arm.hpp"
#include "pegasus_msgs/srv/land.hpp"
#include "pegasus_msgs/msg/status.hpp"

// Messages for showing off the path
#include "nav_msgs/msg/path.hpp"

// Services for path setup
#include "pegasus_msgs/srv/reset_path.hpp"
#include "pegasus_msgs/srv/add_arc.hpp"
#include "pegasus_msgs/srv/add_line.hpp"
#include "pegasus_msgs/srv/add_circle.hpp"
#include "pegasus_msgs/srv/add_lemniscate.hpp"
#include "pegasus_msgs/srv/add_waypoint.hpp"

// Services for starting a path following/tracking mission
#include "dense.hpp"
#include "pegasus_msgs/srv/start_mission.hpp"
#include "pegasus_msgs/srv/land_mission.hpp"

class PathsNode : public rclcpp::Node { 

public: 

    using SharedPtr = std::shared_ptr<PathsNode>;
    using UniquePtr = std::unique_ptr<PathsNode>;
    using WeakPtr = std::weak_ptr<PathsNode>;

    /**
     * @brief Construct a new PathsNode object
     * @param node_name The ROS2 node name
     * @param intra_process_comms Whether to use interprocess communication framework or not (false by default)
     */
    PathsNode(const std::string & node_name, bool intra_process_comms = false);

    /**
     * @brief Destroy the Paths Node object
     */
    ~PathsNode();

private:

    /**
     * @defgroup initFunctions 
     * This group defines all the private initialization functions
     * that will initialize all the ROS publishers, services and actions
     */

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 publishers
     */
    void init_publishers();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 subscribers
     */
    void init_subscribers();

    /**
     * @ingroup initFunctions
     * @brief Method used to initialize all the ROS2 services
     */
    void init_services();

    /**
     * @defgroup servicesCallbacks
     * This group defines all the service server callbacks,
     * such as arming/disarming the vehicle or auto-landing
     */

    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr request, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response);
    
    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void add_arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, const pegasus_msgs::srv::AddArc::Response::SharedPtr response);
    
    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void add_line_callback(const pegasus_msgs::srv::AddLine::Request::SharedPtr request, const pegasus_msgs::srv::AddLine::Response::SharedPtr response);
    
    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void add_circle_callback(const pegasus_msgs::srv::AddCircle::Request::SharedPtr request, const pegasus_msgs::srv::AddCircle::Response::SharedPtr response);
    
    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void add_lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, const pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request
     * @param response
     */
    void add_waypoint_callback(const pegasus_msgs::srv::AddWaypoint::Request::SharedPtr request, const pegasus_msgs::srv::AddWaypoint::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief TODO
     * @param request 
     * @param response 
     */
    void start_mission_callback(const pegasus_msgs::srv::StartMission::Request::SharedPtr request, const pegasus_msgs::srv::StartMission::Response::SharedPtr response);

    /**
     * @ingroup servicesCallbacks
     * @brief Service callback responsible for taking action when a landing mission is requested. By default it will always use the onboard
     * controller for this type of mission
     * @param request A pointer to the request object (unused)
     * @param response A pointer to the response object. Contains whether the landing mission will be executed or not, and a string which might contain
     * or not some information regarding the landing of the vehicle
     */
    void land_mission_callback(const pegasus_msgs::srv::LandMission::Request::SharedPtr request, const pegasus_msgs::srv::LandMission::Response::SharedPtr response);

    /**
     * @defgroup subscribers ROS2 Subscribers
     * This group defines all the ROS subscribers
     */

    /**
     * @brief Subscriber for a pegasus_msg::Status which essentialy will check whether the vehicle is armed/disarmed
     * and landed/flying
     */
    rclcpp::Subscription<pegasus_msgs::msg::Status>::SharedPtr status_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Subscriber to the current state of the vehicle. Useful when the service used to generate
     * a line is a waypoint and the path is empty. Basically, if the path is empty and we request a waypoint,
     * this waypoint will be a line between the current vehicle position and the desired point
     */
    rclcpp::Subscription<pegasus_msgs::msg::State>::SharedPtr state_sub_{nullptr};

    /**
     * @defgroup subscribersCallbacks
     * This group defines all the callbacks used by the ROS2 subscribers
     */

    /**
     * @brief Method that is called by "status_sub_" and updates the variables "armed_" and "flying_"
     * @param msg A pegasus status message
     */
    void status_callback(const pegasus_msgs::msg::Status::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Callback called by "state_sub_" used to update the current position of the vehicle
     * @param msg A message with the current state of the vehicle
     */
    void state_callback(const pegasus_msgs::msg::State::SharedPtr msg);

    /**
     *  @defgroup publishers ROS2 Publishers
     *  This group defines all the ROS publishers
     */

    /**
     * @ingroup publishers 
     * @brief Publisher for a nav_msgs::Path which essentially contains a list of points that describe
     * the path approximately (usefull for visualization in tools such as Rviz)
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr points_pub_{nullptr};

    /**
     * @brief The message that will be published by "points_pub_" and will contain a set of
     * points that give a course representation of the parametric path
     */
    nav_msgs::msg::Path path_points_msg_;

    /**
     * @brief The sample step of the parametric values used to get a list of points that
     * describes the path
     */
    double sample_step_;

    /**
     * @brief The rate at which the control should run
     */
    double control_rate_;

    /**
     * @defgroup services ROS2 Services
     * This group defines all the ROS services
     */

    /**
     * @ingroup services
     * @brief Service server to reset the path
     */
    rclcpp::Service<pegasus_msgs::srv::ResetPath>::SharedPtr reset_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to add an arc to the path
     */
    rclcpp::Service<pegasus_msgs::srv::AddArc>::SharedPtr add_arc_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to add an line to the path
     */
    rclcpp::Service<pegasus_msgs::srv::AddLine>::SharedPtr add_line_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to add a circle to the path
     */
    rclcpp::Service<pegasus_msgs::srv::AddCircle>::SharedPtr add_circle_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to add an line to the path
     */
    rclcpp::Service<pegasus_msgs::srv::AddLemniscate>::SharedPtr add_lemniscate_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to add a waypoint to the path
     */
    rclcpp::Service<pegasus_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to start a path following/tracking mission
     */
    rclcpp::Service<pegasus_msgs::srv::StartMission>::SharedPtr start_mission_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server to start a landing mission
     */
    rclcpp::Service<pegasus_msgs::srv::LandMission>::SharedPtr land_mission_service_{nullptr};

    /**
     * @ingroup services
     * @brief Service server
     */
    rclcpp::Service<pegasus_msgs::srv::Arm>::SharedPtr arm_service_{nullptr};

    /**
     * @brief Auxiliar method that should be called by the services to add a new section to the path
     * @param section A shared pointer to a path section
     */
    void add_section_to_path(const Pegasus::Paths::Section::SharedPtr section);

    /**
     * @brief A path object pointer where each section will be saved
     */
    Pegasus::Paths::Path::SharedPtr path_{nullptr};

    /**
     * @brief The controller to be used to perform a given mission
     */
    BaseControllerNode::SharedPtr controller_{nullptr};

    /**
     * @brief The current vehicle position (usefull when the first path section requested is waypoint)
     * and we generate a line from the current vehicle position to the waypoint
     */
    Eigen::Vector3d vehicle_pos_{0.0, 0.0, 0.0};

    /**
     * @brief Checked true if the vehicle is already armed
     */
    bool armed_{false};

    /**
     * @brief Checked true if the vehicle is already flying (TODO - IMPLEMENT THESE SAFETIES LATER)
     */
    bool flying_{false};
};