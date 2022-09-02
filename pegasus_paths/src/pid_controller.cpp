#include "pid_controller.hpp"
#include "pegasus_utils/rotations.hpp"
#include "thrust_curves/acceleration_to_attitude.hpp"

/**
 * @brief Construct a new Pid Controller object
 * @param nh The nodehandler shared pointer for the base class that creates the controller object
 * @param path A shared pointer for a path the controller must track
 * @param controller_rate The rate at which the controller will operate
 */
PidController::PidController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate) : BaseControllerNode(nh, path, controller_rate) {

    // Read from ROS parameter server, the control gains 
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.kp", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.kd", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.ki", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.kff", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.min_output", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("pid_node.gains.max_output", std::vector<double>());

    auto kp = nh_->get_parameter("pid_node.gains.kp").as_double_array();
    auto kd = nh_->get_parameter("pid_node.gains.kd").as_double_array();
    auto ki = nh_->get_parameter("pid_node.gains.ki").as_double_array();
    auto kff = nh_->get_parameter("pid_node.gains.kff").as_double_array();
    auto min_output = nh_->get_parameter("pid_node.gains.min_output").as_double_array();
    auto max_output = nh_->get_parameter("pid_node.gains.max_output").as_double_array();

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    // Initialize the publisher for the desired vehicle attitude and thrust
    nh_->declare_parameter<std::string>("pid_node.topics.publishers.control", "control/attitude_force");
    control_pub_ = nh_->create_publisher<pegasus_msgs::msg::AttitudeThrustControl>(nh_->get_parameter("pid_node.topics.publishers.control").as_string(), 1);

    // Initialize the publisher for the statistics of the PID controller
    nh_->declare_parameter<std::string>("pid_node.topics.publishers.statistics", "statistics/pid");
    statistics_pub_ = nh_->create_publisher<pegasus_msgs::msg::PidStatistics>(nh_->get_parameter("pid_node.topics.publishers.statistics").as_string(), 1);
}


/**
 * @brief Destroy the Pid Controller object
 */
PidController::~PidController() {}

/**
 * @brief Method to start the path following controller
 */
void PidController::start() {

    // Call the base start function to start the update callback timer
    BaseControllerNode::start();
}

/**
 * @brief Method for stoping the path following controller
 */
void PidController::stop() {

    // Call the base stop function to stop the update callback timer
    BaseControllerNode::stop();
}

/**
 * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
 * "current_velocity_".
 * @param msg A message with the state of the vehicle
 */
void PidController::update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg) {
    // ----------------------------------------
    // Save the current position of the vehicle
    // ----------------------------------------

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

    // Call the base class function to save the pointer to the state of the vehicle
    BaseControllerNode::update_state_callback(msg);
}

/**
 * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
 * which is used to update the control signals
 */
void PidController::controller_update() {

    // Get the time difference between the last call and current call
    double dt = (nh_->get_clock()->now() - prev_time_).seconds();
    
    // Get the path desired position, velocity and acceleration
    auto pd = path_->pd(gamma_);
    auto d_pd = path_->d_pd(gamma_);
    auto dd_pd = path_->dd_pd(gamma_);

    // If the path had a reference, then compute the errors based on that value
    Eigen::Vector3d pos_error, vel_error, accel;
    if(pd.has_value() && d_pd.has_value() && dd_pd.has_value()) {
        // Compute the position error and velocity error using the path desired position and velocity
        pos_error = pd.value() - current_position_;
        vel_error = d_pd.value() - current_velocity_;
        accel = dd_pd.value();
    }
    
    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    for(unsigned int i=0; i < 3; i++) u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], accel[i] * mass_, dt);

    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, desired_yaw_);

    // Publish the control output
    attitude_thrust_msg_.attitude[0] = Pegasus::Rotations::rad_to_deg(attitude_thrust[0]);
    attitude_thrust_msg_.attitude[1] = Pegasus::Rotations::rad_to_deg(attitude_thrust[1]);
    attitude_thrust_msg_.attitude[2] = Pegasus::Rotations::rad_to_deg(attitude_thrust[2]);
    attitude_thrust_msg_.thrust = attitude_thrust[3];
    control_pub_->publish(attitude_thrust_msg_);

    // Publish the statistics message
    update_statistics_msg();
    statistics_pub_->publish(statistics_msg_);

    // Update the prev_time variable
    prev_time_ = nh_->get_clock()->now();
}

/**
 * @brief Method that is called by the "timer_callback" to fill the PID statistics for the x, y and z position pids
 */
void PidController::update_statistics_msg() {

    // For each PID control [x, y, z]
    for(unsigned int i = 0; i < 3; i++) {

        // Get the statistics from the controller object
        Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

        statistics_msg_.statistics[i].dt = stats.dt;
        // Fill the feedback errors
        statistics_msg_.statistics[i].error_p = stats.error_p;
        statistics_msg_.statistics[i].error_d = stats.error_d;
        statistics_msg_.statistics[i].integral = stats.integral;
        statistics_msg_.statistics[i].ff_ref = stats.ff_ref;

        // Fill the errors scaled by the gains
        statistics_msg_.statistics[i].p_term = stats.p_term;
        statistics_msg_.statistics[i].d_term = stats.d_term;
        statistics_msg_.statistics[i].i_term = stats.i_term;
        statistics_msg_.statistics[i].ff_term = stats.ff_term;

        // Fill the outputs of the controller
        statistics_msg_.statistics[i].anti_windup_discharge = stats.anti_windup_discharge;
        statistics_msg_.statistics[i].output_pre_sat = stats.output_pre_sat;
        statistics_msg_.statistics[i].output = stats.output;
    }   
}