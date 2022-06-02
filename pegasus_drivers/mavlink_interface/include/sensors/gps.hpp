#pragma once

#include <mavsdk/plugins/telemetry/telemetry.h>

/**
 * @brief Class that implements the function that subscribes to GPS mavlink topics
 * and 
 */
class GPS {

    public:
        /**
         * @brief Construct a new GPS object
         * 
         * @param telemetry A mavsdk telemetry object
         */
        GPS(mavsdk::Telemetry& telemetry);

        /**
         * @brief Destroy the GPS object
         */
        ~GPS();

        /**
         * @brief Method that subscribes to the GPS mavlink topics
         */
        void subscribe_mavlink_topics();
};