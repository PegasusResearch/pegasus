#pragma once

#include <autopilot/mode.hpp>
#include "paths/path.hpp"

// Services for path setup
#include "pegasus_msgs/srv/reset_path.hpp"
#include "pegasus_msgs/srv/add_arc.hpp"
#include "pegasus_msgs/srv/add_line.hpp"
#include "pegasus_msgs/srv/add_circle.hpp"
#include "pegasus_msgs/srv/add_lemniscate.hpp"

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

    // Publisher of the path to be displayed in RVIZ
    nav_msgs::msg::Path path_points_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr points_pub_{nullptr};    

    // The path for the vehicle to follow    
    Pegasus::Paths::Path path_;
};
}