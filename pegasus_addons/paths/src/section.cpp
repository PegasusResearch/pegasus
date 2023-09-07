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
#include <cmath>
#include "paths/speeds/speed.hpp"
#include "paths/section.hpp"

namespace Pegasus::Paths {

/** 
 * @brief Default method for computing the curvature. The default implementation
 * implements the general formula to compute the curvature based on the derivative
 * equations of the path
 * @param gamma The path parameter
 * @return A double with the path curvature 
 */
double Section::curvature(double gamma) {

    // Saturate the path parameter
    double t = this->limit_gamma(gamma);

    // Compute the first and second derivatives
    Eigen::Vector3d d_pd = this->d_pd(t);
    Eigen::Vector3d dd_pd = this->dd_pd(t);

    // Use the generic formula for the curvature
    return (d_pd[0] * dd_pd[1] - d_pd[1] * dd_pd[0]) / std::pow(d_pd.norm(), 3);
}

/** 
 * @brief Default method for computing the torsion. The default implementation
 * implements the general formula to compute the curvature based on the derivative
 * equations of the path
 * @param gamma The path parameter
 * @return A double with the path torsion 
 */
double Section::torsion(double gamma) {
    //TODO
    (void) gamma;
    return 0.0;
}

/**
 * @brief Default method for computing the tangent angle to the path 
 * @param gamma The path parameter
 * @return A double with the angle of the tangent to the path expressed in radians
 */
double Section::tangent_angle(double gamma) {
    
    // Saturate the path parameter
    double t = this->limit_gamma(gamma);

    // Compute the first derivative
    Eigen::Vector3d d_pd = this->d_pd(t);
    
    /* Compute the tangent to the 2D path */
    return std::atan2(d_pd[1], d_pd[0]);
}

/**
 * @brief Default method for computing the norm of the derivative 
 * @param gamma  The path parameter
 * @return A double with the norm of the derivative of the path position pd
 */
double Section::derivative_norm(double gamma) {
    
    // Saturate the path parameter
    double t = this->limit_gamma(gamma);
    return this->d_pd(t).norm();
}

/**
 * @brief Default method for getting the desired vehicle speed for a particular 
 * location in the path section (in m/s)
 * @param gamma The path parameter
 * @return double The desired vehicle speed (in m/s)
 */
double Section::vehicle_speed(double gamma) {

    // Saturate the path parameter
    double t = this->limit_gamma(gamma);
    
    // Return the desired vehicle speed in m/s
    return vehicle_speed_->get_vehicle_speed(t, *this);
}

/**
 * @brief Default method for getting the desired speed for the evolution
 * of the parametric value
 * @param The path parameter
 * @return double The desired speed progression of the parametric variable
 */
double Section::vd(double gamma) {

    // Saturate the path parameter
    double t = this->limit_gamma(gamma);
    
    // Return the desired speed for the parametric value
    return vehicle_speed_->get_vd(t, *this);
}

/**
 * @brief Default method for getting the desired acceleration for the evolution
 * of the parametric value
 * @param gamma The path parameter
 * @return double The speed progression of the parametric variable
 */
double Section::d_vd(double gamma) {

    // Saturate the path parameter
    double t = this->limit_gamma(gamma);
    
    // Return the desired acceleration for the parametric value
    return vehicle_speed_->get_d_vd(t, *this);
}

/**
 * @brief Method to get a set of sampled points from the path
 * @param step_size The step size to increment the parametric value gamma for sampling purposes. Smaller
 * gammas yield a finner result but also more points
 * @return std::vector<Eigen::Vector3d> A vector of 3d points
 */
std::vector<Eigen::Vector3d> Section::get_samples(double step_size) {

    // Get the number of samples we will produce
    int num_samples = (max_gamma_ - min_gamma_) / step_size;

    // Create an empty vector of samples with the desired size
    std::vector<Eigen::Vector3d> samples(num_samples);

    // Generate the samples and return 
    double curr_gamma = min_gamma_;
    for(int i = 0; i < num_samples; i++) {
        samples[i] = pd(curr_gamma);
        curr_gamma += step_size;
    }

    return samples;
}

}