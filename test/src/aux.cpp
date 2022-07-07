#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <chrono>

#include "api.hpp"

/**
 * @brief Get the system object from mavsdk once the vehicle is detected
 * 
 * @param mavsdk An object of the type mavsdk
 * @return std::shared_ptr<mavsdk::System> A shared pointer to the Vehicle System object
 */
std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk) {
    
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}