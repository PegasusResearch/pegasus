#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "api.hpp"
#include <thread>

void initialize_mavlink_subscribers(mavsdk::Telemetry& telemetry)
{
    // Subscribe to the current arm state and flight mode of the vehicle
    telemetry.subscribe_armed([](bool is_armed) { std::cout << (is_armed ? "armed" : "disarmed") << std::endl; });
    telemetry.subscribe_flight_mode([](mavsdk::Telemetry::FlightMode flight_mode) {std::cout << flight_mode << std::endl; });

    // Subscribe to the battery level state
    telemetry.subscribe_battery([](mavsdk::Telemetry::Battery bat) {std::cout << bat << std::endl; });

    // Subscribe to the GPS state of the vehicle
    telemetry.subscribe_gps_info([](mavsdk::Telemetry::GpsInfo gps_info) {std::cout << gps_info << std::endl; });
    telemetry.subscribe_ground_truth([](mavsdk::Telemetry::GroundTruth gps_data) {std::cout << gps_data << std::endl; });

    // Subscribe to the raw data from the imu
    telemetry.subscribe_imu([](mavsdk::Telemetry::Imu imu) {std::cout << imu << std::endl; });

    // Subscribe to the filtered attitude of the vehicle
    telemetry.subscribe_attitude_quaternion([](mavsdk::Telemetry::Quaternion quat) {std::cout << quat << std::endl; });
    telemetry.subscribe_attitude_euler([](mavsdk::Telemetry::EulerAngle euler) {std::cout << euler << std::endl; });
    telemetry.subscribe_attitude_angular_velocity_body([](mavsdk::Telemetry::AngularVelocityBody ang_vel) {std::cout << ang_vel << std::endl; });

    // Subscribe to the actuator state of the vehicle
    telemetry.subscribe_actuator_control_target([](mavsdk::Telemetry::ActuatorControlTarget act_control_target) {std::cout << act_control_target << std::endl; });
    telemetry.subscribe_actuator_output_status([](mavsdk::Telemetry::ActuatorOutputStatus act_output) {std::cout << act_output << std::endl; });

    telemetry.subscribe_odometry([](mavsdk::Telemetry::Odometry odom) {std::cout << odom << std::endl; });
}

int main(int argc, char ** argv) {

    // Initialize the MAVSDK object and try to connect to the vehicle
    mavsdk::Mavsdk mavsdk;
    std::cout << "Adding Connection" << std::endl;
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14550");
    std::cout << "Connection added" << std::endl;

    // Check if connection failed
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    // Get a pointer to the current vehicle or just terminate the program
    std::shared_ptr<mavsdk::System> system = get_system(mavsdk);
    if (!system) return 1;

    std::cout << "Vehicle detected" << std::endl;

    // Instantiate the mavsdk plugins for the vehicle
    auto action = mavsdk::Action{system};           // Enable simple actions such as arming, taking off, and landing.
    auto offboard = mavsdk::Offboard{system};       // Control a drone with position, velocity, attitude or motor commands.
    auto telemetry = mavsdk::Telemetry{system};     // (e.g. battery, GPS, RC connection, flight mode etc.) and set telemetry update rates.

    // Subscribe to the arm state of the vehicle 
    initialize_mavlink_subscribers(telemetry);

    bool test = true;

    while (test) {
        auto result = action.arm();
        std::cout << result << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}