#include "console_node.hpp"

ConsoleNode::ConsoleNode() : rclcpp::Node("pegasus_console") {

    // Initialize the subscribers, services and publishers
    initialize_subscribers();
    initialize_services();
}

ConsoleNode::~ConsoleNode() {}

void ConsoleNode::initialize_subscribers() {

    // Status of the vehicle
    status_sub_ = this->create_subscription<pegasus_msgs::msg::Status>(
        "/drone1/status2", 
        rclcpp::SensorDataQoS(), 
        [this](const pegasus_msgs::msg::Status::ConstSharedPtr msg) {
            printf("Received status message\n");
            //console_ui_.update_status(msg);
        });
}

void ConsoleNode::initialize_services() {
}

void ConsoleNode::start() {

    // Add this node to the multithread executor
    executor_.add_node(this->shared_from_this());

    // Start the executor in a separate thread
    executor_thread_ = std::thread([this]() {this->executor_.spin();});

    // Start the console UI in this thread
    console_ui_.loop();
}