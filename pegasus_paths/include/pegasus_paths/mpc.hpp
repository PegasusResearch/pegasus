#pragma once

#include <casadi/casadi.hpp>
#include "base_controller.hpp"
#include "pegasus_msgs/msg/attitude_rate_thrust_control.hpp"
#include "pegasus_msgs/srv/thrust_curve.hpp"

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
     * @brief Method used to integrate the system dynamics
     */
    casadi::SX f(const casadi::SX &x, const casadi::SX &u);

    // Time information
    double T_;          /*! @brief The time horizon of the controllers */
    double dt_;         /*! @brief The rate at which the controller works */
    int N_;             /*! @brief The number of input and state samples to generate = (int) T/dt_ */
    
    double g_{9.81};    /*! @brief The gravity acceleration constant m/s^2 @ingroup system_gains */
    
    // State dimensions
    int x_dim_{10};     /*! @brief The dimension of the state vector */
    int u_dim_{4};      /*! @brief The dimension of the input vector */

    // The current state of the quadrotor
    std::vector<double> curr_state_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> ref_state_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> parameters_{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    // Quadrotor state bounds
    std::vector<double> x_min_; 
    std::vector<double> x_max_; 
    
    // Quadrotor input bounds vector
    std::vector<double> u_min_; 
    std::vector<double> u_max_; 

    casadi::SX Q_;   /*! @brief The cost matrix for tracking the end goal point */
    casadi::SX R_;   /*! @brief The cost matrix for the action */

    // Initialization for the CASADi IOPT solver - initial state of the vehicle (warm initialization)
    std::vector<double> x0_;  /*! @brief The initial state of the system */
    std::vector<double> u0_;  /*! @brief The initial control input vector */
    
    std::vector<double> w0_;    /*! @brief The vector used as a warm-start (guess for the optimization variables) */
    std::vector<double> v0_;    

    std::vector<double> lbg_;   /*! @brief The lower-bound for the g function vector (i.e. lbg <= g ) */
    std::vector<double> ubg_;   /*! @brief The upper-bound for the g function vector (i.e. g <= ubg) */
    std::vector<double> lbx_;   /*! @brief The lower-bound for the x state (i.e. lbx <= x) */
    std::vector<double> ubx_;   /*! @brief The upper-bound for the x state (i.e. x <= ubx) */

    /**
     * @brief Function that will actually be solved each time
     */
    casadi::Function solver_;
    casadi::DMDict solver_args_;

    /**
     * @brief Control message to be published periodically by the timer_callback() when
     * the controller is running
     */
    pegasus_msgs::msg::AttitudeRateThrustControl attitude_rate_thrust_msg_;

    /**
     * @brief Publisher for the attitute-rates and total thrust force (in N) using the "attitude_rate_thrust_msg_"
     */
    rclcpp::Publisher<pegasus_msgs::msg::AttitudeRateThrustControl>::SharedPtr control_pub_{nullptr};
};