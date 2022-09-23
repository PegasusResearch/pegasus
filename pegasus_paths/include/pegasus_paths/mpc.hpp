#pragma once

#include <casadi/casadi.hpp>
#include "base_controller.hpp"

class MPCController : public BaseControllerNode {

public:

    using SharedPtr = std::shared_ptr<MPCController>;
    using UniquePtr = std::unique_ptr<MPCController>;
    using WeakPtr = std::weak_ptr<MPCController>;

    /**
     * @brief Construct a new MPC Controller object
     * @param nh The nodehandler shared pointer for the base class that creates the controller object
     * @param path A shared pointer for a path the controller must track
     * @param controller_rate The rate at which the controller will operate
     */
    MPCController(const rclcpp::Node::SharedPtr nh, const Pegasus::Paths::Path::SharedPtr path, const double controller_rate);

    /**
     * @brief Destroy the Pid Controller object
     */
    ~MPCController();

    /**
     * @brief Method to start the path following controller
     */
    void start();

    /**
     * @brief Method for stoping the path following controller. IF THE VEHICLE IS STILL IN THE AIR WHEN THIS
     * METHOD IS CALLED, THEN THE CONTROLLER WILL STOP AND THE VEHICLE MAY FALL, UNLESS THE ONBOARD MICROCONTROLLER
     * HAS SOME SAFETY FEATURE IMPLEMENTED
     */
    void stop();

    /**
     * @brief Method that is called whenever the reference path to follow object is reset. This method should
     * make sure that whenever the path is reset, the vehicle DOES NOT FALL and holds it's position
     */
    void reset();

    /**
     * @brief Method that is called by "state_sub_" to update the variables "current_position_", 
     * "current_velocity_".
     * @param msg A message with the state of the vehicle
     */
    void update_state_callback(const pegasus_msgs::msg::State::SharedPtr msg);

    /**
     * @brief Method that is called periodically by "timer_" when active at a rate "timer_rate_"
     * which is used to update the control signals
     */
    void controller_update();

    /**
     * @brief Get the identifier object Method that should be implemented by a derived class 
     * that returns a string that uniquely identifies the type of controller
     * @return std::string A string that uniquely identifies the type of controller
     */
    virtual inline std::string get_identifier() { return std::string("mpc"); }

private:

    /**
     * @brief Auxiliary function called inside the "controller_update" to update the references that the controller will
     * track with information from the "path_" object
     */
    void update_references();

    /**
     * @brief Auxiliar method used to setup the system dynamics that will be used throughout each iteration of the MPC
     * controller
     */
    void init_dynamics();

    /**
     * @defgroup current_vehicle_state
     * This group defines all variables that represent the vehicle current state
     */
    Eigen::Matrix<double, 10, 1> curr_state_; /*! @brief The current real state of the vehicle @ingroup current_vehicle_state */

    /**
     * @defgroup system_gains
     * This section defines the system gains variables
     */

    double g_{9.81};                        /*! @brief The gravity acceleration constant m/s^2 @ingroup system_gains */
    Eigen::Matrix<double, 10, 10> Q_goal_;  /*! @brief The cost matrix for tracking the end goal point @ingroup system_gains */
    Eigen::Matrix<double, 4, 4> Q_action_;  /*! @brief The cost matrix for the action @ingroup system_gains */

    /**
     * @defgroup initial_conditions
     * This section defines the initial conditions for the controller
     */
    Eigen::Matrix<double, 10, 1> x0_; /*! @brief The initial state of the system @ingroup initial_conditions */
    Eigen::Matrix<double, 4, 1> u0_;  /*! @brief The initial control input vector @ingroup initial_conditions */

    /**
     * @brief Method used to integrate the system dynamics
     * @param dt The time diference between function calls
     * @return casadi::Function A casadi function
     */
    casadi::Function system_dynamics(double dt);

    /**
     * @defgroup input_states
     * This group defines the input states of the controller
     */

    /**
     * @ingroup input_states
     * @brief The state vector of the system [px, py, pz, qw, qx, qy, qz, vx, vy, vz]
     * where pos = [px, py, pz], q = [qw, qx, qy, qz] and v = [vx, vy, vz] (the velocity of the vehicle
     * in the inertial frame)
     */
    Eigen::Matrix<casadi::SX, 10, 1> x_;
    Eigen::Matrix<casadi::SX, 10, 1> x_dot_;
    Eigen::Matrix<casadi::SX, 4, 1> u_;

    casadi::SX a;

    /**
     * @brief The function that defines the differential equation of the system and the 
     * integrated version that returns the state
     */
    casadi::Function f_;
    casadi::Function F_;

    /**
     * @brief Cost functions
     */
    casadi::Function f_cost_goal_;
    casadi::Function f_cost_u_;

    double T_; /*! @brief The time horizon of the controllers */
    double dt_; /*! @brief The rate at which the controller works */
    int N_; /*! @brief The number of input and state samples to generate = (int) T/dt_ */

    /**
     * @defgroup loss_function
     * This group defines the loss function used by the MPC controller
     */

    /**
     * @ingroup loss_function
     * @brief 
     */
    //casadi::Function f{"cost_goal", };

    /**
     * @defgroup system_dynamics
     * This group defines the system dynamics variables and functions
     */

    casadi::MX x_dot{casadi::MX::sym("x_dot", 10, 1)};

    /**
     * @ingroup system_dynamics
     * @brief 
     */
    //casadi::Function f{};
};