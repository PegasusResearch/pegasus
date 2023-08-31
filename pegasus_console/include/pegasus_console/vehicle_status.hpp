#pragma once

#include <map>
#include <array>
#include <string>
#include <Eigen/Dense>

const std::map<uint8_t, std::string> fmu_status_map = {
    {0, "DISARMED"},
    {1, "ARMED"},
    {2, "KILL SWITCH ENGAGED"},
    {3, "POSITION HOLD ACTIVE"},
    {4, "LANDING"},
    {5, "LANDED"},
    {6, "ERROR"}
};

const std::map<uint8_t, std::string> fmu_landed_state_map = {
    {0, "UNKNOWN"},
    {1, "ON GROUND"},
    {2, "IN AIR"},
    {3, "TAKING OFF"},
    {4, "LANDING"}
};

const std::map<uint8_t, std::string> fmu_flight_mode_map = {
    {0, "UNKNOWN"},
    {1, "READY"},
    {2, "TAKEOFF"},
    {3, "HOLD"},
    {4, "MISSION"},
    {5, "RETURN TO LAUNCH"},
    {6, "LAND"},
    {7, "OFFBOARD"},
    {8, "FOLLOW_ME"},
    {9, "MANUAL"},
    {10, "ATL_CTL"},
    {11, "POS_CTL"},
    {12, "ACRO"},
    {13, "STABILIZED"},
    {14, "RATTITUDE"}
};

struct State {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d attitude_euler{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_inertial{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct Battery {
    float temperature{0.0};
    float voltage{0.0};
    float current{0.0};
    float percentage{0.0};
    float amps_hour_consumed{0.0};
};

struct Health {
    bool is_armable{false};
    bool acc_calibrated{false};
    bool mag_calibrated{false};
    bool local_position_ok{false};
    bool global_position_ok{false};
    bool home_position_ok{false};
};

struct RcStatus {
    bool available{false};
    float signal_strength{0.0};
};

struct FmuStatus {
    uint8_t system_id{0};
    bool armed{false};
    uint8_t landed_state{0};
    uint8_t flight_mode{0};
    Battery battery;
    Health health;
    RcStatus rc_status;
};

struct PositionControlWidgetData {
    std::array<std::string, 4> inputs{"", "", "", ""};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    float yaw{0.0};
};

struct ThrotleWidgetData {
    std::string input{""};
    float throtle{0.0};
};

struct AutopilotwidgetData {

    // ----------------------------------
    // Waypoint UI data
    // ----------------------------------
    std::array<std::string, 3> waypoint_pos_input{"", "", ""};
    std::string waypoint_yaw_input{""};

    Eigen::Vector3d waypoint{Eigen::Vector3d::Zero()};
    float waypoint_yaw{0.0};

    // ----------------------------------
    // Arc UI data
    // ----------------------------------
    std::array<std::string, 5> arc_pos_input{"", "", "", "", ""};
    std::string arc_speed_input{""};

    Eigen::Vector<double, 5> arc{Eigen::Vector<double, 5>::Zero()};
    float arc_speed{0.0};

    // ----------------------------------
    // Line UI data
    // ----------------------------------
    std::array<std::string, 6> line_pos_input{"", "", "", "", "", ""};
    std::string line_speed_input{""};

    Eigen::Vector<double, 6> line{Eigen::Vector<double, 6>::Zero()};
    float line_speed{0.0};

    // ----------------------------------
    // Circle UI data
    // ----------------------------------
    std::array<std::string, 4> circle_pos_input{"", "", "", ""};
    std::string circle_speed_input{""};

    Eigen::Vector<double, 4> circle{Eigen::Vector<double, 4>::Zero()};
    float circle_speed{0.0};

    // ----------------------------------
    // Lemniscate UI data
    // ----------------------------------
    std::array<std::string, 4> lemniscate_pos_input{"", "", "", ""};
    std::string lemniscate_speed_input{""};

    Eigen::Vector<double, 4> lemniscate{Eigen::Vector<double, 4>::Zero()};
    float lemniscate_speed{0.0};
};