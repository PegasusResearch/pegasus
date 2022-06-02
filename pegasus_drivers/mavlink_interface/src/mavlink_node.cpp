#include <memory>
#include <mavsdk/mavsdk.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "mavlink_interface/mavlink_node.hpp"

MAVLinkNode::MAVLinkNode() : Node("mavlink_node") {
    
    // Initialize the MAVSDK object and try to connect to the vehicle
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adding Vehicle Connection at udp://:14550");
    mavsdk::ConnectionResult connection_result = this->mavsdk_.add_any_connection("udp://:14550");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection added");

    // Check if connection failed
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        //return 1;
    }

    // Get a pointer to the current vehicle or just terminate the program
    std::shared_ptr<mavsdk::System> system = this->get_system();
    //if (!system) return 1;

    std::cout << "Vehicle detected" << std::endl;
}

MAVLinkNode::~MAVLinkNode() {
    // TODO
}

/**
 * @brief Get the system object from mavsdk once the vehicle is detected
 * 
 * @return std::shared_ptr<mavsdk::System> A shared pointer to the Vehicle System object
 */
std::shared_ptr<mavsdk::System> MAVLinkNode::get_system() {
    
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    this->mavsdk_.subscribe_on_new_system([&prom, this]() {
        auto system = this->mavsdk_.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            this->mavsdk_.subscribe_on_new_system(nullptr);
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

int main(int argc, char ** argv) {

    // Initiate ROS2 library
    rclcpp::init(argc, argv);

    // Create a node to send and receive data between MAVLink and ROS2
    rclcpp::spin(std::make_shared<MAVLinkNode>());
    rclcpp::shutdown();
    return 0;
}