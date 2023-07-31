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
    Eigen::Vector3d position;
    Eigen::Vector3d attitude_euler;
    Eigen::Vector3d velocity_inertial;
    Eigen::Vector3d angular_velocity;
};

struct Battery {
    float temperature;
    float voltage;
    float current;
    float percentage;
    float amps_hour_consumed;
};

struct Health {
    bool is_armable;
    bool acc_calibrated;
    bool mag_calibrated;
    bool local_position_ok;
    bool global_position_ok;
    bool home_position_ok;
};

struct RcStatus {
    bool available;
    float signal_strength;
};

struct FmuStatus {
    uint8_t system_id;
    bool armed;
    uint8_t landed_state;
    uint8_t flight_mode;
    Battery battery;
    Health health;
    RcStatus rc_status;
};