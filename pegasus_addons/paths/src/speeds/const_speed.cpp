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
#include "paths/speeds/const_speed.hpp"
#include "paths/section.hpp"

namespace Pegasus::Paths {

/**
 * @brief Get the desired speed progression for the path parametric value
 * @param gamma The path parametric value
 * @param section A reference to a section the speed is associated with
 * @return double The desired speed progression for the path parametric value
 */
double ConstSpeed::get_vd(double gamma, Pegasus::Paths::Section & section) {
    
    // Define a zero velocity variable to start
    double vd = 0.0;

    // Compute the derivative norm
    double derivative_norm = section.derivative_norm(gamma);

    // Convert the speed from the vehicle frame to the path frame
    if(derivative_norm != 0) vd = vehicle_speed_ / derivative_norm;

    // If the speed exploded because the derivative norm was hill posed, then set it to a very small value
    if(!std::isfinite(vd)) vd = 0.00000001;

    return vd;
}

/**
 * @brief Get the desired acceleration progression for the path parametric value
 * @param gamma The path parametric value
 * @param section A reference to a section the speed is associated with
 * @return double The desired acceleration progression for the path parametric value
 */
double ConstSpeed::get_d_vd(double gamma, Pegasus::Paths::Section & section) {
    (void) gamma;
    (void) section;
    return 0.0;
}

/**
 * @brief Get the vehicle speed progression (in m/s)
 * @param gamma The path parametric value
 * @param section A reference to a section the speed is associated with
 * @return double The desired acceleration progression for the path parametric value
 */
double ConstSpeed::get_vehicle_speed(double gamma, Pegasus::Paths::Section & section) {
    (void) gamma;
    (void) section;
    return vehicle_speed_;
}
}