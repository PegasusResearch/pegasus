#include "lifecycle_service_client.hpp"

/**
 * @brief Construct a new Lifecycle Service Client object
 * @param lifecycle_node_name TODO
 * @param parent_node TODO
 */
LifecycleServiceClient::LifecycleServiceClient(const std::string & lifecycle_node_name, rclcpp::Node::SharedPtr parent_node) : node_(parent_node), change_state_(lifecycle_node_name + "/change_state", node_), get_state_(lifecycle_node_name + "/get_state", node_) {
  
  // Block until server is up
  rclcpp::Rate r(20);
  while (!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}


bool LifeCycleServiceClient::LifecycleServiceClient::change_state(const uint8_t transition, const seconds timeout) {
  
  if (!change_state_.wait_for_service(timeout)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto response = change_state_.invoke(request, timeout);
  return response.get();
}

bool LifeCycleServiceClient::LifecycleServiceClient::change_state(std::uint8_t transition) {
  
  if (!change_state_.wait_for_service(5s)) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

uint8_t LifeCycleServiceClient::LifecycleServiceClient::get_state(const seconds timeout) {
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}