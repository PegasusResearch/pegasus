#include "mpc.hpp"
#include <vector>

/**
 * @brief Construct a new MPC Controller object
 * @param nh The nodehandler shared pointer for the base class that creates the controller object
 * @param path A shared pointer for a path the controller must track
 * @param controller_rate The rate at which the controller will operate
 */
MPCController::MPCController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate) : BaseControllerNode(nh, path, controller_rate) {
    
    // Read from ROS parameter server, the control gains 
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.Q_goal", std::vector<double>({100.0, 100.0, 100.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.Q_action", std::vector<double>({0.1, 0.1, 0.1, 0.1}));
    nh_->declare_parameter<double>("controllers.mpc.gains.T", 2.0);

    Q_goal_ = Eigen::Matrix<double, 10, 10>(nh_->get_parameter("controllers.mpc.gains.Q_goal").as_double_array().data());
    Q_action_ = Eigen::Matrix<double, 4, 4>(nh_->get_parameter("controllers.mpc.gains.Q_action").as_double_array().data());

    // Save the rate of the controller to use in the MPC and the time horizon
    dt_ = controller_rate;
    T_ = nh_->get_parameter("controllers.mpc.gains.T").as_double();

    // Compute the number of system samples to compute at each iteration
    N_ = static_cast<int>(T_ / dt_);

    a = casadi::SX::sym("A", 2, 2);
}

/**
 * @brief Destroy the Pid Controller object
 */
MPCController::~MPCController() {
    //TODO
}

/**
 * @brief Method to start the path following controller
 */
void MPCController::start() {

    // Initialize the initial state variables
    // TODO: check if this is performing a deep or shallow copy
    x0_ = curr_state_;
    u0_ = Eigen::Matrix<double, 4, 1>({0.0, 0.0, 0.0, -g_});
    
    // Initialize the MPC controller with the correct initial conditions of the vehicle
    init_dynamics();
}

/**
 * @brief Method for stoping the path following controller. IF THE VEHICLE IS STILL IN THE AIR WHEN THIS
 * METHOD IS CALLED, THEN THE CONTROLLER WILL STOP AND THE VEHICLE MAY FALL, UNLESS THE ONBOARD MICROCONTROLLER
 * HAS SOME SAFETY FEATURE IMPLEMENTED
 */
void MPCController::stop() {
    //TODO
    return;
}

/**
 * @brief Method that is called whenever the reference path to follow object is reset. This method should
 * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
 */
void MPCController::reset() {
    //TODO
    return;
}

/**
 * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
 * "current_velocity_".
 * @param msg A message with the state of the vehicle
 */
void MPCController::update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg) {
    
    // Update the current position of the vehicle
    curr_state_[0] = msg->pose.pose.position.x;
    curr_state_[1] = msg->pose.pose.position.y;
    curr_state_[2] = msg->pose.pose.position.z;

    // Update the current orientation of the vehicle
    curr_state_[3] = msg->pose.pose.orientation.w;
    curr_state_[4] = msg->pose.pose.orientation.x;
    curr_state_[5] = msg->pose.pose.orientation.y;
    curr_state_[6] = msg->pose.pose.orientation.z;

    // Update the current velocity of the vehicle in the inertial frame
    curr_state_[7] = msg->inertial_vel.vector.x;
    curr_state_[8] = msg->inertial_vel.vector.y;
    curr_state_[9] = msg->inertial_vel.vector.z;
}

/**
 * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
 * which is used to update the control signals
 */
void MPCController::controller_update() {
    // TODO
    return;
}

/**
 * @brief Auxiliary function called inside the "controller_update" to update the references that the controller will
 * track with information from the "path_" object
 */
void MPCController::update_references() {
    //TODO
    return;
}  

/**
 * @brief Auxiliar method used to setup the system dynamics that will be used throughout each iteration of the MPC
 * controller
 */
void MPCController::init_dynamics() {
    
    // --------------------
    // SYSTEM DYNAMICS
    // --------------------
    //       0, 1, 2,  3,  4,  5,  6,  7,  8,  9
    // x_ = [x, y, z, qw, qx, qy, qz, vx, vy, vz]
    // u_ = [thrust, wx, wy, wz]

    // Derivative of the position equals the velocity
    x_dot_[0] = x_[7]; 
    x_dot_[1] = x_[8];
    x_dot_[2] = x_[9];

    // Derivative of the quaternion
    x_dot_[3] = 0.5 * ( -u_[1]*x_[4] - u_[2]*x_[5] - u_[3]*x_[6] );
    x_dot_[4] = 0.5 * (  u_[1]*x_[3] + u_[3]*x_[5] - u_[2]*x_[6] );
    x_dot_[5] = 0.5 * (  u_[2]*x_[3] - u_[3]*x_[4] + u_[1]*x_[6] );
    x_dot_[6] = 0.5 * (  u_[3]*x_[3] + u_[2]*x_[4] - u_[1]*x_[5] );

    // Derivative of the velocity
    x_dot_[7] = 2 * ( x_[3]*x_[5] + x_[4]*x_[6] ) * -u_[0];
    x_dot_[8] = 2 * ( x_[5]*x_[6] - x_[3]*x_[4] ) * -u_[0];
    x_dot_[9] = (x_[3]*x_[3] - x_[4]*x_[4] -x_[5]*x_[5] + x_[6]*x_[6]) * -u_[0] + g_;

    // Concatenate the input and state variables into a list
    casadi::SXVector function_input(u_.data(), u_.data() + u_.size());
    casadi::SXVector function_state(x_.data(), x_.data() + x_.size());
    function_input.insert(function_input.end(), std::make_move_iterator(function_state.begin()), std::make_move_iterator(function_state.end()));

    // Get the x_dot vector as a casadi vector
    casadi::SXVector function_output(x_dot_.data(), x_dot_.data() + x_dot_.size());

    // Create the differential equation that governs the evolution of the state
    f_ = casadi::Function("f", function_input, function_output, casadi::Dict());

    // Create the function that integrates the x_dot over a period of time dt (the rate of the controller)
    F_ = system_dynamics(dt_);
    F_.map(N_, "openmp"); // parallel execution

    // --------------------
    // LOSS FUNCTION
    // --------------------


    // ----------------------
    // NONLINEAR OPTIMIZATION
    // ----------------------


}

/**
 * @brief Method used to integrate the system dynamics
 * @param dt The time diference between function calls
 * @return casadi::Function A casadi function
 */
casadi::Function MPCController::system_dynamics(double dt) {

    (void) dt;
    // double DT = dt / 4.0;

    // casadi::MX X0 = casadi::MX::sym("X", self._s_dim);
    // casadi::MX U = casadi::MX::sym("U", self._u_dim);

    // X = X0

    // // Perform RK4 integration
    // for(int i = 0; i < 4; i++) {
    //     k1 =DT*self.f(X, U)
    // }
    return casadi::Function();
}