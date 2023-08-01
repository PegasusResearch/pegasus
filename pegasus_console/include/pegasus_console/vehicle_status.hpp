#pragma once

#include <map>
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