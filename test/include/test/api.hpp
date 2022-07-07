#pragma once 
#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <memory>

/**
 * @brief Get the system object from mavsdk once the vehicle is detected
 * 
 * @param mavsdk An object of the type mavsdk
 * @return std::shared_ptr<mavsdk::System> A shared pointer to the Vehicle System object
 */
std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk);