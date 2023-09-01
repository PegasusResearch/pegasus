#pragma once

#include <autopilot/mode.hpp>

// Library that implements parameterized paths to follow
#include "paths/dense.hpp"

// Library that implements PID controllers
#include "pid/pid.hpp"

// Services for path setup
#include "pegasus_msgs/srv/reset_path.hpp"
#include "pegasus_msgs/srv/add_arc.hpp"
#include "pegasus_msgs/srv/add_line.hpp"
#include "pegasus_msgs/srv/add_circle.hpp"
#include "pegasus_msgs/srv/add_lemniscate.hpp"

// Messages for controller debugging
#include "pegasus_msgs/msg/pid_statistics.hpp"

#include "nav_msgs/msg/path.hpp"

namespace autopilot {

class FollowTrajectoryMode : public autopilot::Mode {

public:

    ~FollowTrajectoryMode();

    void initialize() override;
    virtual bool enter();
    virtual bool exit() override;
    virtual void update(double dt);

protected:

    // Get the desired position, velocity and acceleration from the path
    void update_reference(double dt);
    bool check_finished();
    void update_statistics();

    // Auxiliar function to add a section to the path
    void add_section_to_path(const Pegasus::Paths::Section::SharedPtr section);

    // Services callbacks to set the path to follow
    void reset_callback(const pegasus_msgs::srv::ResetPath::Request::SharedPtr request, const pegasus_msgs::srv::ResetPath::Response::SharedPtr response);
    void add_arc_callback(const pegasus_msgs::srv::AddArc::Request::SharedPtr request, const pegasus_msgs::srv::AddArc::Response::SharedPtr response);
    void add_line_callback(const pegasus_msgs::srv::AddLine::Request::SharedPtr request, const pegasus_msgs::srv::AddLine::Response::SharedPtr response);
    void add_circle_callback(const pegasus_msgs::srv::AddCircle::Request::SharedPtr request, const pegasus_msgs::srv::AddCircle::Response::SharedPtr response);
    void add_lemniscate_callback(const pegasus_msgs::srv::AddLemniscate::Request::SharedPtr request, const pegasus_msgs::srv::AddLemniscate::Response::SharedPtr response);
    
    // Services to set the path
    rclcpp::Service<pegasus_msgs::srv::ResetPath>::SharedPtr reset_service_{nullptr};
    rclcpp::Service<pegasus_msgs::srv::AddArc>::SharedPtr add_arc_service_{nullptr};
    rclcpp::Service<pegasus_msgs::srv::AddLine>::SharedPtr add_line_service_{nullptr};
    rclcpp::Service<pegasus_msgs::srv::AddCircle>::SharedPtr add_circle_service_{nullptr};
    rclcpp::Service<pegasus_msgs::srv::AddLemniscate>::SharedPtr add_lemniscate_service_{nullptr};

    // Publisher for the PID statistics and the path to follow
    nav_msgs::msg::Path path_points_msg_;
    pegasus_msgs::msg::PidStatistics pid_statistics_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr points_pub_{nullptr}; 
    rclcpp::Publisher<pegasus_msgs::msg::PidStatistics>::SharedPtr statistics_pub_{nullptr};

    // The step to sample the path
    double sample_step_{0.1};

    // The mass of the vehicle
    double mass_;

    // Set the progression speed of the parametric variable
    double gamma_{0.0};
    double d_gamma_{0.0};
    double dd_gamma_{0.0};
    
    // Set the desired targets to follow
    Eigen::Vector3d desired_position_{0.0, 0.0, 0.0};
    Eigen::Vector3d desired_velocity_{0.0, 0.0, 0.0};
    Eigen::Vector3d desired_acceleration_{0.0, 0.0, 0.0};
    double desired_yaw_{0.0};

    // The PID controller to follow the path
    std::array<Pegasus::Pid::UniquePtr, 3> controllers_{nullptr};

    // The path for the vehicle to follow    
    Pegasus::Paths::Path path_;
};
}