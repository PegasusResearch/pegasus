#pragma once

#include <memory>
#include <limits>

namespace Pegasus {

/**
 * @brief Class that implement the generic, boring, old school but good enough PID controller
 */
class Pid {

public:

    /**
     * @brief An alias for a shared pointer of a PID controller
     */
    using SharedPtr = std::shared_ptr<Pid>;

    /**
     * @brief An alias for a unique pointer of a PID controller
     */
    using UniquePtr = std::unique_ptr<Pid>;

    /**
     * @brief An alias for a weak pointer of a PID controller
     */
    using WeakPtr = std::weak_ptr<Pid>;


    struct Statistics {

        /* The last time step in seconds taken between the current and last iteration */
        double dt{0.0};

        /* Errors and references not affected by the gains */
        double error_p{0.0};        /**< @brief The proportional error fed into the p_term */
        double error_d{0.0};        /**< @brief The derivative error fed into the d_term */
        double integral{0.0};       /**< @brief The integral value fed into the i_term */
        double ff_ref{0.0};         /**< @brief The fead-forward value fed into the ff_term */
        
        /* PID terms already afftected by the gains */
        double p_term{0.0};         /**< @brief The scaled proportional error (kp * error_p) */
        double d_term{0.0};         /**< @brief The scaled derivative error (kd * error_d) */
        double i_term{0.0};         /**< @brief The scaled integral value (i_term <=> iterm += ki * (error_p - prev_error_p)) */
        double ff_term{0.0};        /**< @brief The scaled feed-forward value (kff * ff_ref) */
        
        /* Outputs of the control loop */
        double anti_windup_discharge{0.0}; /**< @brief The amount that was discharged from the integral via the anti-windup mechanism */
        double output_pre_sat{0.0};        /**< @brief The output of the PID before being saturated */
        double output{0.0};                /**< @brief The actual output of the PID */    
    };

    /**
     * @brief Construct a new Pid controller object
     * @param kp The proportional gain
     * @param kd The derivative gain
     * @param ki The integral gain
     * @param kff The feed-forward gain
     * @param min_out the minimum output reference allowed
     * @param max_out the maximum output reference allowed
     */
    Pid(double kp, double kd, double ki, double kff, double min_out, double max_out) : 
        kp_(kp), kd_(kd), ki_(ki), kff_(kff), min_output_(min_out), max_output_(max_out) {}

    /**
     * @brief Destroy the Pid controller object
     */
    ~Pid() {}

    /**
     * @brief Method to update the output of the PID controller. Note, when this method is invoked,
     * the derivative part of the control is computed numerically based on the previous value of the error.
     * No filtering is applied, so be aware of possible shuttering effect.
     * 
     * @param error_p The error between the reference value and the actual value
     * @param feed_forward_ref The reference used to multiply by the feed-forward term
     * @param dt The time delay between the previous function call and the next function call (in seconds)
     * @return double the output of the PID controller
     */
    double compute_output(double error_p, double feed_forward_ref, double dt);

    /**
     * @brief Method to update the output of the PID controller
     * @param error_p The error between the reference value and the actual value
     * @param error_d The derivative of the error. I.e. pref_dot - p_dot.
     * @param feed_forward_ref The reference used to multiply by the feed-forward term
     * @param dt The time delay between the previous function call and the next function call (in seconds)
     * @return double the output of the PID controller
     */
    double compute_output(double error_p, double error_d, double feed_forward_ref, double dt);

    /**
     * @brief Method used to get the reference to the statistics structure object in which the 
     * statics of the controller are saved for debugging and performance analysis purposes
     * @return Statistics& A reference to the stats_ private object
     */
    inline Statistics& get_statistics() {return stats_;}

    /**
     * @brief Method that is used to reset the pid controller variables
     */
    void reset_controller();

private:

    /**
     * @brief Structure that will hold the computed errors and outputs of the controller for debuging purposes
     */
    Statistics stats_;

    /**
     * @defgroup gains 
     * This section defines the controller gains
     */

    /**
     * @ingroup gains
     * @brief The proportional gain of the controller
     */
    double kp_;

    /**
     * @ingroup gains
     * @brief The derivative gain of the controller
     */
    double kd_;

    /**
     * @ingroup gains
     * @brief The integral gain of the controller
     */
    double ki_;

    /**
     * @ingroup gains
     * @brief The feed-forward gain of the controller
     */
    double kff_;

    /**
     * @defgroup saturations
     * This sections defines all the saturations variables
     */

    /**
     * @ingroup saturations
     * @brief The minimum output allowed
     */
    double min_output_;

    /**
     * @ingroup saturations
     * @brief The maximum output allowed
     */
    double max_output_;

    /**
     * @defgroup auxiliaryVariables
     * This section defines all auxiliary variables used
     * by the controller between iterations
     */
    
    /**
     * @brief The previous value of the reference error
     */
    double prev_error_p_{0.0};

    /**
     * @brief The acumulated value of the integral
     */
    double error_i_{0.0};

};

}