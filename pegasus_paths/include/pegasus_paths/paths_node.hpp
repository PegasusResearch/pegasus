#pragma once

#include "paths/dense.hpp"
#include "rclcpp/rclcpp.hpp"

// Messages for the state of the vehicle and the gamma value
#include "std_msgs/msg/float64.hpp"
#include "pegasus_msgs/msg/state.hpp"

// Messages for showing off the path
#include "nav_msgs/msg/path.hpp"
#include "pegasus_msgs/msg/path.hpp"

// Services for path setup
#include "pegasus_msgs/srv/reset_path.hpp"
#include "pegasus_msgs/srv/add_arc.hpp"
#include "pegasus_msgs/srv/add_line.hpp"
#include "pegasus_msgs/srv/add_circle.hpp"
#include "pegasus_msgs/srv/add_lemniscate.hpp"
#include "pegasus_msgs/srv/add_waypoint.hpp"

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
     * that will initialize all the ROS publishers, subscribers, services and actions
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
     * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
     */
    void timer_callback();

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
     * @defgroup subscriberCallbacks
     * This group defines all the ROS subscriber callbacks
     */
    
    /**
     * @ingroup subscriberCallbacks
     * @brief Callback called by "state_sub_" used to update the current position of the vehicle
     * @param msg A message with the current state of the vehicle
     */
    void state_callback(const pegasus_msgs::msg::State::SharedPtr msg);

    /**
     * @ingroup subscriberCallbacks
     * @brief Callback called by "gamma_sub_" used to update the current path parametric value (gamma)
     * @param msg A message with a float with the current value of gamma
     */
    void gamma_callback(const std_msgs::msg::Float64::SharedPtr msg);

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
     * @ingroup publishers
     * @brief Publisher for the actual values of the parametric path evaluated at a given parametric
     * value gamma
     */
    rclcpp::Publisher<pegasus_msgs::msg::Path>::SharedPtr path_pub_{nullptr};

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
     * @brief The message that will be published by "path_pub_" and will contain all control
     * data of the path evaluated at a given parametric value gamma
     */
    pegasus_msgs::msg::Path path_msg_;

    /**
     * @defgroup subscribers ROS2 Subscribers
     * This group defines all the ROS subscribers
     */

    /**
     * @ingroup subscribers
     * @brief Subscriber to the current state of the vehicle. Useful when the service used to generate
     * a line is a waypoint and the path is empty. Basically, if the path is empty and we request a waypoint,
     * this waypoint will be a line between the current vehicle position and the desired point
     */
    rclcpp::Subscription<pegasus_msgs::msg::State>::SharedPtr state_sub_{nullptr};

    /**
     * @ingroup subscribers
     * @brief Subscriber to the current path parameter value (gamma) - a.k.a virtual target on the path
     * that will be used as a reference for position and path following controllers
     */
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gamma_sub_{nullptr};

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
     * @brief Auxiliar method that should be called by the services to add a new section to the path
     * @param section A shared pointer to a path section
     */
    void add_section_to_path(const Pegasus::Paths::Section::SharedPtr section);

    /**
     * @brief Timer used to make the controller run at a constant rate
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief The rate at which the timer will call the timer_callback, expressed in Hz
     */
    double timer_rate_;

    /**
     * @brief A path object where each section will be saved
     */
    Pegasus::Paths::Path path_;

    /**
     * @brief The current path parametric value
     */
    double gamma_{0.0};

    /**
     * @brief The current vehicle position
     */
    Eigen::Vector3d vehicle_pos_{0.0, 0.0, 0.0};
};