#include "pid_node.hpp"
#include "pegasus_utils/rotations.hpp"
#include "pegasus_msgs/srv/thrust_curve.hpp"
#include "thrust_curves/acceleration_to_attitude.hpp"

/**
 * @brief Constructor for the PID controller node
 * @param node_name The ROS2 node name
 * @param intra_process_comms Whether to use interprocess communication framework or not (false by default)
 */
PidNode::PidNode(const std::string & node_name, bool intra_process_comms) : 
    rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {

    // Read the rate at which the node will operate from the parameter server
    declare_parameter<double>("pid_node.rate", 10.0);
    timer_rate_ = get_parameter("pid_node.rate").as_double();

    // Read from ROS parameter server, the control gains 
    declare_parameter<std::vector<double>>("pid_node.gains.kp", std::vector<double>());
    declare_parameter<std::vector<double>>("pid_node.gains.kd", std::vector<double>());
    declare_parameter<std::vector<double>>("pid_node.gains.ki", std::vector<double>());
    declare_parameter<std::vector<double>>("pid_node.gains.kff", std::vector<double>());
    declare_parameter<std::vector<double>>("pid_node.gains.min_output", std::vector<double>());
    declare_parameter<std::vector<double>>("pid_node.gains.max_output", std::vector<double>());

    auto kp = get_parameter("pid_node.gains.kp").as_double_array();
    auto kd = get_parameter("pid_node.gains.kd").as_double_array();
    auto ki = get_parameter("pid_node.gains.ki").as_double_array();
    auto kff = get_parameter("pid_node.gains.kff").as_double_array();
    auto min_output = get_parameter("pid_node.gains.min_output").as_double_array();
    auto max_output = get_parameter("pid_node.gains.max_output").as_double_array();

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) {
        controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);
    }

    // Request the mass of the vehicle - note, this part is locking, which means the controller will not initiate if
    // the driver providing the mass of the vehicle is also not available (which kind of makes sense)
    // You don't want a controller to work if it does not know the mass of the vehicle. After all, this is a flying lawm mower
    declare_parameter<std::string>("pid_node.topics.services.mass", "thrust_curve");
    auto mass_client = create_client<pegasus_msgs::srv::ThrustCurve>(get_parameter("pid_node.topics.services.mass").as_string());
    
    while(!mass_client->wait_for_service()) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for mass information service. Exiting.");
        }
    }

    mass_ = mass_client->async_send_request(std::make_shared<pegasus_msgs::srv::ThrustCurve::Request>()).get()->mass;
}


// --------------------------------------------------
// In this section we define the methods for managing
// the state machine transition of the Lifecycle node
// --------------------------------------------------
  
/**
 * @defgroup state_machine_callbacks
 * This section defines all the callbacks that are responsible for transitions in the node state machine
 */

/**
 * @ingroup state_machine_callbacks
 * @brief on_activate callback is being called when the lifecycle node
 * enters the "activating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "active" state or stays
 * in "inactive".
 * TRANSITION_CALLBACK_SUCCESS transitions to "active"
 * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PidNode::on_activate(const rclcpp_lifecycle::State & state) {

    // Call the default on_activate method
    LifecycleNode::on_activate(state);

    // Reset the initialization variables (wait for the subscriptions to set these variables to true)
    state_initialized_ = false;
    reference_initialized_ = false;

    // Initialize the subscriber to the state of the vehicle
    declare_parameter<std::string>("pid_node.topics.subscribers.state", "nav/state");
    state_sub_ = create_subscription<pegasus_msgs::msg::State>(get_parameter("pid_node.topics.subscribers.state").as_string(), 1, std::bind(&PidNode::update_state_callback, this, std::placeholders::_1));

    // Initialize the subscriber to the desired reference to follow
    declare_parameter<std::string>("pid_node.topics.subscribers.path", "path");
    path_sub_ = create_subscription<pegasus_msgs::msg::Path>(get_parameter("pid_node.topics.subscribers.path").as_string(), 1, std::bind(&PidNode::update_reference_callback, this, std::placeholders::_1));

    // Initialize the publisher for the desired vehicle attitude and thrust
    declare_parameter<std::string>("pid_node.topics.publishers.control", "control/attitude_force");
    control_pub_ = this->create_publisher<pegasus_msgs::msg::AttitudeThrustControl>(get_parameter("pid_node.topics.publishers.control").as_string(), 1);

    // Cleanup the PID controllers if they were used before
    for(unsigned int i=0; i < 3; i++) {
        if(controllers_[i]) controllers_[i]->reset_controller();
    }

    // Update the last time that the controller was called to now (initialization)
    prev_time_ = get_clock()->now();

    // Initialize the periodic timer
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&PidNode::timer_callback, this));

    // Return success
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/**
 * @ingroup state_machine_callbacks
 * @brief on_deactivate callback is being called when the lifecycle node
 * enters the "deactivating" state.
 * Depending on the return value of this function, the state machine
 * either invokes a transition to the "inactive" state or stays
 * in "active".
 * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
 * TRANSITION_CALLBACK_FAILURE transitions to "active"
 * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PidNode::on_deactivate(const rclcpp_lifecycle::State & state) {
    
    // Call the default on_activate method
    LifecycleNode::on_deactivate(state);

    // Reset the timer that calls the control law periodically
    timer_->cancel();
    timer_->reset();

    // Stop the subscription to the vehicle state (no need for extra overhead when the controller is not running)
    state_sub_.reset();
    control_pub_.reset();
    path_sub_.reset();

    // Stop the publisher for the control law
    control_pub_.reset();    

    // Reset the initialization variables
    state_initialized_ = false;
    reference_initialized_ = false;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// --------------------------------------------------
// In this section we define the methods for actually
// performing the control
// --------------------------------------------------

/**
 * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
 */
void PidNode::timer_callback() {

    // Check if the state of the vehicle was initialized. If not, then 
    // do not send any control scheme and just return and wait
    if(!state_initialized_) return;

    // If the reference is not initialized, but the state of the vehicle is,
    // use the current position of the vehicle as a reference, and it should hold its position
    // statically
    if(!reference_initialized_) update_reference_for_hold_position();

    // Get the time difference between the last call and current call
    double dt = (get_clock()->now() - prev_time_).seconds();

    // Compute the position error and velocity error
    Eigen::Vector3d pos_error = desired_position_ - current_position_;
    Eigen::Vector3d vel_error = desired_velocity_ - current_velocity_;
    
    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    for(unsigned int i=0; i < 3; i++) {
       u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], desired_acceleration_[i] * mass_, dt);
    }

    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, desired_yaw_);

    // Publish the control output
    attitude_thrust_msg_.attitude[0] = Pegasus::Rotations::rad_to_deg(attitude_thrust[0]);
    attitude_thrust_msg_.attitude[1] = Pegasus::Rotations::rad_to_deg(attitude_thrust[1]);
    attitude_thrust_msg_.attitude[2] = Pegasus::Rotations::rad_to_deg(attitude_thrust[2]);
    attitude_thrust_msg_.thrust = attitude_thrust[3];
    control_pub_->publish(attitude_thrust_msg_);

    // Update the prev_time variable
    prev_time_ = get_clock()->now();
}

/**
 * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
 * "current_velocity_".
 * @param msg A message with the state of the vehicle
 */
void PidNode::update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg) {
    
    state_initialized_ = true;

    // Update the current vehicle position, expressed in the inertial frame (NED)
    current_position_[0] = msg->pose.pose.position.x;
    current_position_[1] = msg->pose.pose.position.y;
    current_position_[2] = msg->pose.pose.position.z;

    // Update the current vehicle velocity, expressed in the inertial frame (NED)
    current_velocity_[0] = msg->inertial_vel.vector.x;
    current_velocity_[1] = msg->inertial_vel.vector.y;
    current_velocity_[2] = msg->inertial_vel.vector.z;

    // Update the current yaw of the vehicle
    current_yaw_ = Pegasus::Rotations::deg_to_rad(msg->rpy.vector.z);
}

/**
 * @brief Method that is called by "path_sub_" to update the variables "desired_position_",
 * "desired_velocity_", "desired_acceleration_" and "desired_yaw_"
 * @param msg A message with the desired path to follow
 */
void PidNode::update_reference_callback(const pegasus_msgs::msg::Path::SharedPtr msg) {

    reference_initialized_ = true;

    for(unsigned int i=0; i < 3; i++) {
        // Update the desired position to follow
        desired_position_[i] = msg->pd[i];

        // Update the desired velocity 
        desired_velocity_[i] = msg->d_pd[i];

        // Update the desired acceleration
        desired_acceleration_[i] = msg->dd_pd[i];
    }
    
    // Update the desired yaw reference for the vehicle
    desired_yaw_ = Pegasus::Rotations::deg_to_rad(msg->yaw);
}

/**
 * @brief Method that is called by the "timer_callback" to set the position reference to the current
 * vehicle position. The same applies for the yaw angle. The desired velocity and acceleration
 * are set to zero
 */
void PidNode::update_reference_for_hold_position() {
    
    reference_initialized_ = true;

    for(unsigned int i=0; i < 3; i++) {
        // Update the desired position to follow
        desired_position_[i] =current_position_[i];

        // Set the desired velocity and acceleration to 0.0
        desired_velocity_[i] = 0.0;
        desired_acceleration_[i] = 0.0;
    }

    // Update the desired yaw reference for the vehicle
    desired_yaw_ = current_yaw_;
}