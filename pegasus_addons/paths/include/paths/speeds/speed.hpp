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
#pragma once

#include <memory>
#include "./../section.fwd.hpp"

namespace Pegasus::Paths {

class Speed {

public:

    using SharedPtr = std::shared_ptr<Speed>;
    using UniquePtr = std::unique_ptr<Speed>;
    using WeakPtr = std::weak_ptr<Speed>;

    /**
     * @brief Get the desired speed progression for the path parametric value
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired speed progression for the path parametric value
     */
    virtual double get_vd(double gamma, Section & section) = 0;

    /**
     * @brief Get the desired acceleration progression for the path parametric value
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired acceleration progression for the path parametric value
     */
    virtual double get_d_vd(double gamma, Section & section) = 0;

    /**
     * @brief Get the vehicle speed progression (in m/s)
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired acceleration progression for the path parametric value
     */
    virtual double get_vehicle_speed(double gamma, Section & section) = 0;

    /**
     * @brief Destructor of the Speed base class. This destructor is virtual
     * such that inherited classes do not invoke this destructor when invoking
     * the delete keyword using a BaseClass pointer
     */
    virtual ~Speed() {}

protected:

    /**
     * @brief Empty Constructor for a base Speed. This method cannot be invoked
     * externally, which means it can only be invoked by the constructor of other derived classes
     */
    Speed() {}
};

}