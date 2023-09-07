/****************************************************************************
 *
 *   Copyright (C) 2023 Marcelo Jacinto. All rights reserved.
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Pegasus nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "pid/pid.hpp"

namespace Pegasus {

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
double Pid::compute_output(double error_p, double feed_forward_ref, double dt) {
    
    // Compute the derivative of the error numerically
    double error_d = (error_p - prev_error_p_) / dt;

    // Invoke the PID control
    return compute_output(error_p, error_d, feed_forward_ref, dt);
}

/**
 * @brief Method to update the output of the PID controller
 * @param error_p The error between the reference value and the actual value
 * @param error_d The derivative of the error. I.e. pref_dot - p_dot.
 * @param feed_forward_ref The reference used to multiply by the feed-forward term
 * @param dt The time delay between the previous function call and the next function call (in seconds)
 * @return double the output of the PID controller
 */
double Pid::compute_output(double error_p, double error_d, double feed_forward_ref, double dt) {

    // Compute the integral term (using euler integration) - TODO: improve the integration part
    error_i_ += ki_ * (error_p * dt);

    // Compute the PID terms
    double p_term = kp_ * error_p;
    double d_term = kd_ * error_d;
    double i_term = error_i_;
    double ff_term = kff_ * feed_forward_ref;

    // Compute the output and saturate it
    double output = p_term + d_term + i_term + ff_term;
    double saturated_ouput = std::max(min_output_, std::min(output, max_output_));

    // Add anti-windup to the integral term (discharge the integral if our output is being saturated)
    double discharge_rate = 0.0;
    if(output > max_output_ || output < min_output_) {

        // Compute the anti-windup discharge rate
        discharge_rate = ki_ * (saturated_ouput - output);

        // Anti-windup effect (discharge the integral)
        error_i_ += discharge_rate;
    }

    // Update the prev error variable
    prev_error_p_ = error_p;

    // Update the statistics structure used for extracting the performance of the control loop
    stats_.dt = dt;
    stats_.error_p = error_p;
    stats_.error_d = error_d;
    stats_.integral = error_i_;
    stats_.ff_ref = feed_forward_ref;
    stats_.p_term = p_term;
    stats_.d_term = d_term;
    stats_.i_term = i_term;
    stats_.ff_term = ff_term;
    stats_.anti_windup_discharge = discharge_rate;
    stats_.output_pre_sat = output;
    stats_.output = saturated_ouput;

    return saturated_ouput;
}

/**
 * @brief Method that is used to reset the pid controller variables
 */
void Pid::reset_controller() {
    prev_error_p_ = 0.0;
    error_i_ = 0.0;
}

}