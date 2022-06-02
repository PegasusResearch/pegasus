#include "sensors/gps.hpp"

GPS::GPS(mavsdk::Telemetry& telemetry) {
    // Subscribe to the GPS state of the vehicle
    telemetry.subscribe_gps_info([](mavsdk::Telemetry::GpsInfo gps_info) {std::cout << gps_info << std::endl; });
    telemetry.subscribe_ground_truth([](mavsdk::Telemetry::GroundTruth gps_data) {std::cout << gps_data << std::endl; });
}

