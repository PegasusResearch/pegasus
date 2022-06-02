#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "api.hpp"
#include <thread>

void subscribe_armed(mavsdk::Telemetry& telemetry)
{
    telemetry.subscribe_armed(
        [](bool is_armed) { std::cout << (is_armed ? "armed" : "disarmed") << '\n'; });

    telemetry.subscribe_battery([](mavsdk::Telemetry::Battery bat) {std::cout << bat << '\n'; });
}

int main(int argc, char ** argv) {

    // Initialize the MAVSDK object and try to connect to the vehicle
    mavsdk::Mavsdk mavsdk;
    std::cout << "Adding Connection" << std::endl;
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udp://192.168.4.2:14550");
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
    subscribe_armed(telemetry);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}