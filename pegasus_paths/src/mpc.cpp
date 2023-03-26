#include "mpc.hpp"
#include "pegasus_utils/rotations.hpp"

#include <chrono>

/**
 * @brief Construct a new MPC Controller object
 * @param nh The nodehandler shared pointer for the base class that creates the controller object
 * @param path A shared pointer for a path the controller must track
 * @param controller_rate The rate at which the controller will operate
 */
MPCController::MPCController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate) : BaseControllerNode(nh, path, controller_rate) {
    
    // Read from ROS parameter server, the control gains, saturations, time, etc.
    nh_->declare_parameter<double>("controllers.mpc.gains.T", 3.0);
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.Q", std::vector<double>({100.0, 100.0, 100.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.R", std::vector<double>({0.1, 0.1, 0.1, 0.1}));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.u_min", std::vector<double>({1.0, -6.0, -6.0, -6.0}));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.u_max", std::vector<double>({20.0, 6.0,  6.0,  6.0}));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.x_min", std::vector<double>(10, -casadi::inf));
    nh_->declare_parameter<std::vector<double>>("controllers.mpc.gains.x_max", std::vector<double>(10, casadi::inf));

    // Save the rate of the controller to use in the MPC and the time horizon
    dt_ = 1.0 / controller_rate;
    T_ = nh_->get_parameter("controllers.mpc.gains.T").as_double();
    N_ = static_cast<int>(T_ / dt_);

    // Define the penalty matrices for the goal and input effort
    Q_ = casadi::SX::eye(10) * casadi::SX(nh_->get_parameter("controllers.mpc.gains.Q").as_double_array());
    R_ = casadi::SX::eye(4) * casadi::SX(nh_->get_parameter("controllers.mpc.gains.R").as_double_array());

    // Define the input constraints of the system
    u_min_ = std::vector<double>(nh_->get_parameter("controllers.mpc.gains.u_min").as_double_array());
    u_max_ = std::vector<double>(nh_->get_parameter("controllers.mpc.gains.u_max").as_double_array());

    x_min_ = std::vector<double>(nh_->get_parameter("controllers.mpc.gains.x_min").as_double_array());
    x_max_ = std::vector<double>(nh_->get_parameter("controllers.mpc.gains.x_max").as_double_array());

    // Initialize the publisher for the desired vehicle attitude and thrust
    nh_->declare_parameter<std::string>("controllers.mpc.topics.publishers.control", "control/attitude_rate_force");
    control_pub_ = nh_->create_publisher<pegasus_msgs::msg::AttitudeRateThrustControl>(nh_->get_parameter("controllers.mpc.topics.publishers.control").as_string(), 1);

    RCLCPP_INFO_STREAM(nh_->get_logger(), "MPC PUBLISHERS INITIALIZED");
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

    // Intialize the current vehicle position for using in the NMPC controller for the first iteration (for the warm-start)
    x0_ = curr_state_;
    u0_ = std::vector<double>({g_, 0.0, 0.0, 0.0});
    
    // Initialize the MPC controller with the correct initial conditions of the vehicle
    init_dynamics();

    // Call the base start function to start the update callback timer
    BaseControllerNode::start();
}

/**
 * @brief Method for stoping the path following controller. IF THE VEHICLE IS STILL IN THE AIR WHEN THIS
 * METHOD IS CALLED, THEN THE CONTROLLER WILL STOP AND THE VEHICLE MAY FALL, UNLESS THE ONBOARD MICROCONTROLLER
 * HAS SOME SAFETY FEATURE IMPLEMENTED
 */
void MPCController::stop() {
    
    // Call the base stop function to stop the update callback timer
    BaseControllerNode::stop();
}

/**
 * @brief Method that is called whenever the reference path to follow object is reset. This method should
 * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
 */
void MPCController::reset() {
    //TODO - for now do nothing
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

    // Set the reference state manually for testing purposes
    ref_state_[2] = -5.0; // z-pos
    ref_state_[3] = 1.0;  // qw
    
    // NOTE: P:=Parameters of the system = [initial_state, ref_state]
    for(int i=0; i < x_dim_; i++) {
        parameters_[i] = curr_state_[i];
        parameters_[i + x_dim_] = ref_state_[i];
    }

    // Update the initial state to be used in the optimization problem
    solver_args_["p"] = parameters_;

    // Solve the NMPC optimization problem
    casadi::DMDict res = solver_(solver_args_);

    // Get the response
    casadi::DM argmin = res["x"];

    // Update the x0_ to be used as the warm start of the system
    // w0_ = std::vector<double>();
    solver_args_["x0"] = w0_;

    // Apply the control input to the vehicle
    //attitude_rate_thrust_msg_.attitude_rate[0] = Pegasus::Rotations::rad_to_deg(attitude_thrust[0]);
    //attitude_rate_thrust_msg_.attitude_rate[1] = Pegasus::Rotations::rad_to_deg(attitude_thrust[1]);
    //attitude_rate_thrust_msg_.attitude_rate[2] = Pegasus::Rotations::rad_to_deg(attitude_thrust[2]);
    //attitude_thrust_msg_.thrust = thrust * 1.50; //KG
    control_pub_->publish(attitude_rate_thrust_msg_);
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

    std::cout << "INIT DYNAMICS" << std::endl;

    // Define the system input variables, parameters and states
    casadi::SX U = casadi::SX::sym("U", u_dim_, N_);       // Inputs to the system over time [u1, u2, ..., uN]
    casadi::SX P = casadi::SX::sym("P", x_dim_ + x_dim_);  // Parameters of the system [initial_state, ref_state]
    casadi::SX X = casadi::SX::sym("X", x_dim_, N_ + 1);   // The state of the system over time [x0, x1, ..., XN]

    // Note: each column of X corresponds to [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
    // Note: each column of U corresponds to [thrust, wx, wy, wz]
    casadi::SX obj = casadi::SX::zeros(1,1);    // The objective function
    std::vector<casadi::SX> g;                  // Constraint vector (used by the multiple shooting approach)
    
    std::vector<double> lbu;    // The lower-bound for the input vector u (i.e. lbu <= u)
    std::vector<double> ubu;    // The upper-bound for the input vector u (i.e. u <= ubu)

    // Define the lower and upper bounds for the state vector x[:,0]
    for(int i=0; i < x_dim_; i++) {
        lbx_.push_back(x_min_[i]);
        ubx_.push_back(x_max_[i]);
    }

    // Increment the equality constraints used by multiple shooting
    g.push_back(X(casadi::Slice(), 0) - P(casadi::Slice(0, 10)));
    for(int i=0; i < x_dim_; i++) {
        lbg_.push_back(0.0);
        ubg_.push_back(0.0);
    }

    // Add the initial conditions of the vehicle for the solver warm-start
    for(int i=0; i < x_dim_; i++) w0_.push_back(x0_[i]);

    for(int n=0; n < N_; n++) {

        casadi::SX st = X(casadi::Slice(), n);
        casadi::SX con = U(casadi::Slice(), n);

        // Add the initial conditions of the vehicle for the solver warm-start
        for(int i=0; i < x_dim_; i++) w0_.push_back(x0_[i]);
        for(int i=0; i < u_dim_; i++) v0_.push_back(u0_[i]);

        //Increment the objective function
        obj += casadi::SX::mtimes(casadi::SX::mtimes((st - P(casadi::Slice(10,20))).T(), casadi::SX(Q_)), st - P(casadi::Slice(10,20))) + casadi::SX::mtimes(casadi::SX::mtimes(con.T(), casadi::SX(R_)), con);

        // Integrate the state using RK4 method        
        casadi::SX k1 = f(st, con);
        casadi::SX k2 = f(st + dt_/2*k1, con);
        casadi::SX k3 = f(st + dt_/2*k2, con);
        casadi::SX k4 = f(st + dt_*k3, con);
        casadi::SX st_next=st + dt_/6*(k1 + 2*k2 + 2*k3 + k4);

        // Increment the equality constraints used by multiple shooting
        g.push_back(X(casadi::Slice(), n+1) - st_next);
        for(int i=0; i < x_dim_; i++) {
            lbg_.push_back(0.0);
            ubg_.push_back(0.0);
        }

        // Define the lower and upper bounds for the input vector u
        for(int i=0; i < u_dim_; i++) {
            lbu.push_back(u_min_[i]);
            ubu.push_back(u_max_[i]);
        }

        // Define the lower and upper bounds for the state vector x
        for(int i=0; i < x_dim_; i++) {
            lbx_.push_back(x_min_[i]);
            ubx_.push_back(x_max_[i]);
        }
    }

    // Define the optimization variables, by concatenating X and U
    casadi::SX OPT_variables = casadi::SX::vertcat({casadi::SX::reshape(X, x_dim_ * (N_+1), 1), casadi::SX::reshape(U, u_dim_ * N_, 1)});

    // Concatenate the vectors of lower and upper bounds of X and U as well
    for(int i=0; i < lbu.size(); i++) {
        lbx_.push_back(lbu[i]);
        ubx_.push_back(ubu[i]);
    }

    // Concatenate the vector of the warm start for the state X and input u
    for(int i=0; i < v0_.size(); i++) {
        w0_.push_back(v0_[i]);
    }

    // Set the lower and upper bounds of the optimization variables and constraints
    solver_args_["x0"] = w0_;
    solver_args_["lbx"] = lbx_;
    solver_args_["ubx"] = ubx_;
    solver_args_["lbg"] = lbg_;
    solver_args_["ubg"] = ubg_;

    // Define the interior point method solver options
    casadi::SXDict nlp_dict = {{"f", obj}, {"x", OPT_variables}, {"p", P}, {"g", casadi::SX::vertcat(g)}};

    // Create the solver with basic options, using the interior point method
    casadi::Dict ipopt_options = {{"verbose", false}, {"ipopt.max_iter", 100}, {"ipopt.warm_start_init_point", "yes"}, {"ipopt.print_level", 0}, {"print_time", true}};
    solver_ = casadi::nlpsol("solver", "ipopt", nlp_dict, ipopt_options);

    solver_.generate_dependencies("nmpc.c");

    //auto C = casadi::Importer("nmpc.c", "clang");
}


/**
 * @brief Method used to integrate the system dynamics
 */
casadi::SX MPCController::f(const casadi::SX &x, const casadi::SX &u) {

    // The position of the vehicle
    casadi::SX px = x(0); 
    casadi::SX py = x(1); 
    casadi::SX pz = x(2);

    // Quaternion that represents the orientation of the vehicle
    casadi::SX qw = x(3); 
    casadi::SX qx = x(4); 
    casadi::SX qy = x(5); 
    casadi::SX qz = x(6);

    // The velocity of the vehicle in the inertial frame
    casadi::SX vx = x(7); 
    casadi::SX vy = x(8); 
    casadi::SX vz = x(9);

    // The thrust and angle rates inputs of the system
    casadi::SX thrust = u(0); 
    casadi::SX wx = u(1); 
    casadi::SX wy = u(2); 
    casadi::SX wz = u(3);

    // --------------------
    // SYSTEM DYNAMICS
    // --------------------
    // Define the system dynamics vector and x_dot = f(x, u) function
    return casadi::SX::vertcat({
        vx,
        vy,
        vz,
        0.5 * ( -wx*qx - wy*qy - wz*qz ),
        0.5 * (  wx*qw + wz*qy - wy*qz ),
        0.5 * (  wy*qw - wz*qx + wx*qz ),
        0.5 * (  wz*qw + wy*qx - wx*qy ),
        2 * ( qw*qy + qx*qz ) * -thrust,
        2 * ( qy*qz - qw*qx ) * -thrust, 
        (qw*qw - qx*qx -qy*qy + qz*qz) * -thrust + g_
    });
}