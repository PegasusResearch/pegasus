#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/takeoff.hpp"

namespace autopilot {

class TakeoffMode : public autopilot::Mode {

public:

    ~TakeoffMode();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The altitude service callback
    void altitude_callback(const pegasus_msgs::srv::Takeoff::Request::SharedPtr request, const pegasus_msgs::srv::Takeoff::Response::SharedPtr response);

    // The target altitude to takeoff to
    float target_altitude;

    // The target position for the vehicle to take off to
    Eigen::Vector3d takeoff_pos{Eigen::Vector3d::Zero()};
    float takeoff_yaw{0.0f};

    // The takeoff service server that changing the target altitude for the vehicle to takeoff to
    rclcpp::Service<pegasus_msgs::srv::Takeoff>::SharedPtr altitude_service_{nullptr};
};

}