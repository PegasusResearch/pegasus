#include "ros_lifecycle_service_client/lifecycle_service_client.hpp"

/**
 * @brief Construct a new Lifecycle Service Client object
 * @param lifecycle_node_name The name of the node to administrate
 * @param parent_node A shared pointer to the base node the service will be created from
 */
LifeCycleServiceClient::LifeCycleServiceClient(const std::string & lifecycle_node_name, rclcpp::Node::SharedPtr parent_node) : node_(parent_node), change_state_(lifecycle_node_name + "/change_state", node_), get_state_(lifecycle_node_name + "/get_state", node_) {

  using namespace std::chrono_literals;

  // Block until we can get the access to the current state service provided by the lifecycle node
  rclcpp::Rate r(20);
  while(!get_state_.wait_for_service(2s)) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for service %s...", get_state_.getServiceName().c_str());
    r.sleep();
  }
}

/**
 * @brief Trigger a state change
 * @param transition The code of the transition to trigger
 * @param timeout The number of seconds to await for the transition to finish
 * @return bool A boolean whether the transition was processed or not
 * Throws std::runtime_error on failure
*/
bool LifeCycleServiceClient::change_state(const uint8_t transition, const std::chrono::seconds timeout) {
  
  // Try to wait for the change state service availability
  if (!change_state_.wait_for_service(timeout)) {
    throw std::runtime_error("change_state service is not available!");
  }

  // Try to change the state of the lifecycle node with a given timeout
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  // Return the response
  auto response = change_state_.invoke(request, timeout);
  return response.get();
}

/**
 * @brief Trigger a state change, returning result
 * @param transition The code of the transition to trigger
 * @return bool A boolean whether the transition was processed or not
 */
bool LifeCycleServiceClient::change_state(std::uint8_t transition) {

  using namespace std::chrono_literals;
  
  // Try to wait for the change state service availability
  if (!change_state_.wait_for_service(5s)) {
    throw std::runtime_error("change_state service is not available!");
  }

  // Try to change the state of the lifecycle node and block until a response is given
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

/**
 * @brief Get the current state as a lifecycle_msgs::msg::State id value
 * Throws std::runtime_error on failure
 */
uint8_t LifeCycleServiceClient::get_state(const std::chrono::seconds timeout) {

  // Try to wait for the get state service availability
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  // Try to change the state of the lifecycle node with timeout and return the current state of the node
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}