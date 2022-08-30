#include "manager_node.hpp"
#include "rclcpp/logger.hpp"

/**
 * @brief Construct a new Manager Node object
 * @param node_name A string with the standard name of the node
 * @param intra_process_comms Whether to use ROS2 intra-process communication feature (for low over-head communication)
 */
ManagerNode::ManagerNode(const std::string & node_name, bool intra_process_comms) :
    rclcpp::Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {

    // -----------------------------------------
    // Initialize the ROS2 parameters
    // -----------------------------------------

    // The list of names of nodes to be administrated 
    declare_parameter("node_names", std::vector<std::string>());
    node_names_ = get_parameter("node_names").as_string_array();

    // Whether to autostart the managed nodes
    declare_parameter("autostart", false);
    autostart_ = get_parameter("autostart").as_bool();

    // Setup the transition state map
    transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE] = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE] = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;

    // Setup the transition labels
    transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
    transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
    transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
    transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
    transition_label_map_[lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] = std::string("Shutting down ");

    // For now, just guarantee that every node transitions to the active state

}

/**
 * @brief Destroy the Manager Node object
 */
ManagerNode::~ManagerNode() {}

/**
 * @defgroup support_service_calls
 * This section defines all the support functions used to make the service calls
 * that will manage the lifecycle nodes
 */

/**
 * @ingroup support_service_calls
 * @brief Start up managed nodes.
 * @return true or false
 */
bool ManagerNode::startup() {

    // Try to change all nodes from unconfigured state to inactive state
    if(!change_state_of_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        RCLCPP_ERROR(get_logger(), "FAILED TO CONFIGURE ALL LIFECYCLE NODES TO CONFIGURED STATE. ABORT");
        return false;
    }

    // Try to change all nodes from inactive to active state 
    if(!change_state_of_all_nodes(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
        RCLCPP_ERROR(get_logger(), "FAILED TO CONFIGURE ALL LIFECYCLE NODES TO ACTIVE STATE. ABORT");
        return false;
    }   

    RCLCPP_INFO(get_logger(), "ALL LIFECYCLE NODES ARE ACTIVE");
    return true;
}

/**
 * @brief Transition a given node to a desired target state
 * @param node A string with the name of the node
 * @param transition The target transition that enables a desired state
 * @return bool Whether the change was successfull or not
 */
bool ManagerNode::change_state_of_node(const std::string & node, std::uint8_t transition) {
    
    if (transition == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE || 
        transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
        
        // Iterate over all nodes
        for(auto & node: node_names_) {
            try {
                if()
            } catch (const std::runtime_error & e) {

            }
        }
    }
}

/**
 * @brief Transition all the nodes to a desired target state
 * @param transition The target transition that enables the desired state
 * @param hard_transition TODO
 * @return bool Whether the change was successfull or not
 */
bool ManagerNode::change_state_of_all_nodes(std::uint8_t transition, bool hard_transition=false) {

}

void ManagerNode::shutdown_all_nodes();