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
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.kp", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.kd", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.ki", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.kff", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.min_output", std::vector<double>());
    nh_->declare_parameter<std::vector<double>>("controllers.pid.gains.max_output", std::vector<double>());

    auto kp = nh_->get_parameter("controllers.pid.gains.kp").as_double_array();
    auto kd = nh_->get_parameter("controllers.pid.gains.kd").as_double_array();
    auto ki = nh_->get_parameter("controllers.pid.gains.ki").as_double_array();
    auto kff = nh_->get_parameter("controllers.pid.gains.kff").as_double_array();
    auto min_output = nh_->get_parameter("controllers.pid.gains.min_output").as_double_array();
    auto max_output = nh_->get_parameter("controllers.pid.gains.max_output").as_double_array();
    
    if(kp.size() == 0 || kd.size() == 0 || ki.size() == 0 || kff.size() == 0 || min_output.size() == 0 || max_output.size() == 0) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Could not read PID position controller gains correctly from the ROS2 parameter server");
        throw std::runtime_error("Gains vector was empty");
    }
    
    RCLCPP_INFO_STREAM(nh_->get_logger(), "PID gains read from ROS2 parameter server correctly");

    // Create the 3 PID controllers for x, y and z axis
    for(unsigned int i=0; i < 3; i++) controllers_[i] = std::make_unique<Pegasus::Pid>(kp[i], kd[i], ki[i], kff[i], min_output[i], max_output[i]);

    RCLCPP_INFO_STREAM(nh_->get_logger(), "ALLOCATING MEMORY");

    // Initialize the service that requests the total mass of the vehicle to the driver
    nh_->declare_parameter<std::string>("controllers.pid.topics.services.mass", "thrust_curve");
    mass_srv_ = nh_->create_client<pegasus_msgs::srv::ThrustCurve>(nh_->get_parameter("controllers.pid.topics.services.mass").as_string());

    RCLCPP_INFO_STREAM(nh_->get_logger(), "START REQUESTING MASS");

    // TODO - FIX THE MASS SERVICE
    mass_ = 1.5;

    // Send an async request to get the mass of the vehicle
    // auto request_mass = std::make_shared<pegasus_msgs::srv::ThrustCurve::Request>();

    // mass_srv_->async_send_request(request_mass, [this] (rclcpp::Client<pegasus_msgs::srv::ThrustCurve>::SharedFuture future) {
        
    //     // Get the result of the async service
    //     auto result = future.get();

    //     // Update the mass variable asynchronously
    //     this->mass_ = result->mass;

    //     // Notify the user that this initialization was done corretly
    //     RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Mass of vehicle = " << this->mass_ << " inside PID position tracking controller");
    // }).wait();

    // Initialize the publisher for the desired vehicle attitude and thrust
    nh_->declare_parameter<std::string>("controllers.pid.topics.publishers.control", "control/attitude_force");
    control_pub_ = nh_->create_publisher<pegasus_msgs::msg::AttitudeThrustControl>(nh_->get_parameter("controllers.pid.topics.publishers.control").as_string(), 1);

    // Initialize the publisher for the statistics of the PID controller
    nh_->declare_parameter<std::string>("controllers.pid.topics.publishers.statistics", "statistics/pid");
    statistics_pub_ = nh_->create_publisher<pegasus_msgs::msg::PidStatistics>(nh_->get_parameter("controllers.pid.topics.publishers.statistics").as_string(), 1);

    RCLCPP_INFO_STREAM(nh_->get_logger(), "PID PUBLISHERS INITIALIZED");
}


/**
 * @brief Destroy the Pid Controller object
 */
PidController::~PidController() {
    // DO NOTHING - NO MEMORY WAS ALLOCATED

    // Undeclare the ros2 parameters
    nh_->undeclare_parameter("controllers.pid.gains.kp");
    nh_->undeclare_parameter("controllers.pid.gains.kd");
    nh_->undeclare_parameter("controllers.pid.gains.ki");
    nh_->undeclare_parameter("controllers.pid.gains.kff");
    nh_->undeclare_parameter("controllers.pid.gains.min_output");
    nh_->undeclare_parameter("controllers.pid.gains.max_output");
}

/**
 * @brief Method to start the path following controller
 */
void PidController::start() {

    // Check if the mass has a value that makes sense
    if(mass_ <= 0.0) {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Mass of the vehicle invalid. Mass = " << mass_<< ". Unable to start the PID position tracking controller");
    }

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
 * @brief Method that is called whenever the reference path to follow object is reset. This method should
 * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
 */
void PidController::reset() {

    // Reset the current parametric value, such that if the path get's repopulated, then we will start following
    // the new references from the begining
    gamma_ = 0.0;
    gamma_dot_ = 0.0;
    gamma_ddot_ = 0.0;
    vd_ = 0.0;
    min_gamma_ = 0.0;
    max_gamma_ = 0.0;

    // By default we do not need to add any extra logic, because we are always checking at each controller iteration
    // if we have some reference from the path to follow
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

    // Update the position, velocity and acceleration references for the PID to track
    // using the data inside the path, or a hold position value if the path is empty
    update_references();

    // Compute the position error and velocity error using the path desired position and velocity
    Eigen::Vector3d pos_error = desired_position_ - current_position_;
    Eigen::Vector3d vel_error = desired_velocity_ - current_velocity_;
    Eigen::Vector3d accel = desired_acceleration_;
    
    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    for(unsigned int i=0; i < 3; i++) u[i] = controllers_[i]->compute_output(pos_error[i], vel_error[i], accel[i] * mass_, dt);
    
    // Subtract the gravity in the Z-axis
    u[2] -= 9.8;

    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, desired_yaw_);

    // Publish the control output
    attitude_thrust_msg_.attitude[0] = Pegasus::Rotations::rad_to_deg(attitude_thrust[0]);
    attitude_thrust_msg_.attitude[1] = Pegasus::Rotations::rad_to_deg(attitude_thrust[1]);
    attitude_thrust_msg_.attitude[2] = Pegasus::Rotations::rad_to_deg(attitude_thrust[2]);
    attitude_thrust_msg_.thrust = attitude_thrust[3];
    control_pub_->publish(attitude_thrust_msg_);

    // Update the virtual target parametric value (if we have a path)
    gamma_dot_ = vd_;
    gamma_ += gamma_dot_ * dt;

    // Saturate the value of gamma  between the minimum and maximum values
    gamma_ = std::min(std::max(min_gamma_, gamma_), max_gamma_);

    // TODO - diferentiate to obtain the gamma_ddot_ 
    // for now, since the value is always very small, we aproximate it by zero

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
        statistics_msg_.statistics[i].reference = desired_position_[i];
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

/**
 * @brief Auxiliary function called inside the "controller_update" to update the references that the controller will
 * track with information from the "path_" object
 */
void PidController::update_references() {

    // If the path is empty or is just a nullpointer, do not update the system reference 
    // and use the last (so that the vehicle holds its position) 
    if(path_ == nullptr ||  path_->empty() == true) {

        // set the desired references besides the position to zero (HOLD-POSITION)
        desired_velocity_ << 0.0, 0.0, 0.0;
        desired_acceleration_ << 0.0, 0.0, 0.0;
        gamma_dot_ = 0.0;
        gamma_ddot_ = 0.0;
        vd_ = 0.0;
        min_gamma_ = 0.0;
        max_gamma_ = 0.0;
        return;
    }

    // Get the path desired position, velocity and acceleration from the path object
    std::optional<Eigen::Vector3d> pd = path_->pd(gamma_);
    Eigen::Vector3d d_pd = path_->d_pd(gamma_).value_or(Eigen::Vector3d(0.0, 0.0, 0.0));
    Eigen::Vector3d dd_pd = path_->dd_pd(gamma_).value_or(Eigen::Vector3d(0.0, 0.0, 0.0));

    // Update the desired speed progression for the path virtual target
    vd_ = path_->vd(gamma_).value_or(0.0);

    // Check if the optional contained a position;
    if(!pd.has_value()) {
        // do not update the reference position of the vehicle and hold the position (HOLD-POSITION)
        desired_velocity_ << 0.0, 0.0, 0.0;
        desired_acceleration_ << 0.0, 0.0, 0.0;
        gamma_dot_ = 0.0;
        gamma_ddot_ = 0.0;
        vd_ = 0.0;
        min_gamma_ = 0.0;
        max_gamma_ = 0.0;
        return;
    }

    // Update the reference variables for the PID to track
    desired_position_ = pd.value();
    desired_velocity_ = d_pd * gamma_dot_;
    desired_acceleration_ = (dd_pd * std::pow(gamma_dot_, 2)) + (d_pd * std::pow(gamma_ddot_, 2));

    // Update the saturation values for the virtual target
    min_gamma_ = path_->get_min_gamma();
    max_gamma_ = path_->get_max_gamma();
}