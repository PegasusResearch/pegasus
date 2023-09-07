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
#include <string>
#include <vector>
#include <map>


namespace Pegasus {

/**
 * @brief An abstract thrust curve class to be used as template for other thrust curve classes
 */
class ThrustCurve {

public:

    using SharedPtr = std::shared_ptr<ThrustCurve>;

    /**
     * @brief Method that is used to convert a force in Newton (N) to a percentage from 0-100%. This method
     * is virtual and must be implemented by derived classes
     * @param force The force to apply in Newton (N)
     * @return double The percentage of the total force a vehicle can apply, from 0-100%
     */
    virtual double force_to_percentage(double force) = 0;

    /**
     * @brief Method that is used to convert a percentage from 0-100% of total force a vehicle can apply and 
     * convert it to a force in Newton (N). This method is virtual and must implemented by derived classes
     * @param percentage The percentage of the total force a vehicle can apply, from 0-100%
     * @return double The force to apply in Newton (N)
     */
    virtual double percentage_to_force(double percentage) = 0;

    /**
     * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
     * @return double The maximum force the vehicle can apply in Newton (N)
     */
    virtual double get_max_force() = 0;

    /**
     * @brief Get the type of thrust curve that is instantiated
     * @return std::string A string with the name of the thrust curve instantiated
     */
    virtual std::string get_type() = 0;

    /**
     * @brief Get the parameters that define the thrust curve
     * @return std::map A map that defines the thrust curve parameter names and values
     */
    std::map<std::string, double> get_parameters() {return parameters_; }

    /**
     * @brief Destructor of the ThrustCurve base class. This destructor is virtual
     * such that inherited classes do not invoke this destructor when invoking
     * the delete keyword using a BaseClass pointer
     */
    virtual ~ThrustCurve() = 0;

protected:

    /**
     * @brief Empty Constructor for a base Thrust Curve. This method cannot be invoked
     * externally, which means it can only be invoked by the constructor of other derived classes
     */
    ThrustCurve() {};

    /**
     * @brief The thrust curve parameters map
     */
    std::map<std::string, double> parameters_;
};

/**
 * @brief Function pointer to a function which receives a vector with parameters as inputs,
 * and outputs and instance of a given thrust curve object
 */
typedef ThrustCurve::SharedPtr (*ThrustCurveCreator) (std::map<std::string, double>);

class ThrustCurveFactory {

public:

    /**
     * @brief Method used to instantiate a ThrustCurve object
     * @return ThrustCurve::SharedPtr A shared pointer to a ThrustCurve object
     */
    ThrustCurve::SharedPtr create_thrust_curve(std::map<std::string, double>, std::string);

    /**
     * @brief Get the singleton instance object of this thrust curve factory (only one object of this type per program)
     * @return ThrustCurveFactory& A reference to the singleton instance of this object
     */
    static ThrustCurveFactory& get_instance();

    /**
     * @brief Assign a thrust function with its creator 
     * @param _identifier A string which identifies the type of thrust curve that will be saved
     * @param _creator A function that receives a map of parameters <string, double> and a string with an identifier
     * as inputs and returns a pointer to an object of the type ThrustCurve
     * @return bool A boolean whether a ThrustCurve object was registed in the ThrustCurveFactory successfully or not
     */
    bool register_creator(const std::string& _identifier, ThrustCurveCreator _creator);

private:

    /**
     * @brief Constructor for the ThrustCurveFactory is private, hence disabled,
     * because this class is supposed to be used as a factory, hence only the get_instace
     * method should be invoked
     */
    ThrustCurveFactory() {}

    /**
     * @brief Map of each of the registered identifiers and functions that 
     * instantiate a given ThrustCurve
     */
    std::map<std::string, ThrustCurveCreator> creators_;
};

/**
 * @brief Quadratic thrust curve class that derives from thrust curve base class
 */
class QuadraticThrustCurve : public ThrustCurve {

public:

    /**
     * @brief Static method used to instantiate a quadratic thrust curve object
     * @param gains The quadratic gains that define the curve. A map with string {a: double, b: double, c: double: scale:double}
     * that defines the quadratic equation f = a * (x/scale)^2 + b(x/scale) + c, where x is a percentage from 0-100%
     * and f is given in Newton (N). The scale gain is optional, and if not specified it will be defaulted to 1.0
     * @return ThrustCurve A shared pointer to a ThrustCurve object
     */
    static ThrustCurve::SharedPtr create_thrust_curve(std::map<std::string, double> gains);

    /**
     * @brief Method that is used to convert a force in Newton (N) to a percentage from 0-100%. This method
     * is virtual and must be implemented by derived classes
     * @param force The force to apply in Newton (N)
     * @return double The percentage of the total force a vehicle can apply, from 0-100%
     */
    virtual double force_to_percentage(double force);

    /**
     * @brief Method that is used to convert a percentage from 0-100% of total force a vehicle can apply and 
     * convert it to a force in Newton (N). This method is virtual and must implemented by derived classes
     * @param percentage The percentage of the total force a vehicle can apply, from 0-100%
     * @return double The force to apply in Newton (N)
     */
    virtual double percentage_to_force(double percentage);

    /**
     * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
     * @return double The maximum force the vehicle can apply in Newton (N)
     */
    virtual double get_max_force();

    /**
     * @brief Get the type of thrust curve that is instantiated
     * @return std::string A string with the name of the thrust curve instantiated
     */
    virtual std::string get_type() { return IDENTIFIER; };

    /**
     * @brief Destructor of the QuadraticThrustCurve base class. This is empty
     * as no memory allocation is made
     */
    ~QuadraticThrustCurve() {};
    
private:
    
    /**
     * @brief Construct a new Quadratic Thrust Curve object, that will encode a thrust curve of the type
     * force = ax^2+bx+c, with x a percentage between 0-100%
     * @param a The qudratic gain
     * @param b The linear gain
     * @param c The offset
     * @param scale The scale through x is affected. By default is 1.0. Sometimes, the parameters a, b, and c
     * are defined assuming that x varies from [0, 1], hence if in thoses cases we choose scale = 100, then
     * force = a(x/100.0)^2 + b(x/scale) + c
     */
    QuadraticThrustCurve(double a, double b, double c, double scale=1.0);

    /**
     * @brief Quadratic equation terms
     */
    double a_, b_, c_, scale_;

    /**
     * @brief The max force that the vehicle can output in Newton (N)
     */
    double max_force_;

    /**
     * @brief The unique identifier for this thrust curve
     */
    static const std::string IDENTIFIER;

    /**
     * @brief A Bolean that ensures that the thrust
     * curve is registered with the factory at compile time
     */
    static const bool REGISTERED_WITH_FACTORY;
};


/**
 * @brief A thrust curve that is slightly exponential at the origin and approximately linear 
 * at the middle of the curve
 */
class LinearExponentialThrustCurve : public ThrustCurve {

public:

    /**
     * @brief Static method used to instantiate a quadratic thrust curve object
     * @param gains The quadratic gains that define the curve. A map with string {a: double, b: double, c: double, d: double, scale:double}
     * that defines the equation 
     * percentage = (a * exp(b*force) * sqrt(force) + (c * force) + d) * scale
     * @return ThrustCurve* A pointer to a ThrustCurve object
     */
    static ThrustCurve::SharedPtr create_thrust_curve(std::map<std::string, double> gains);

    /**
     * @brief Method that is used to convert a force in Newton (N) to a percentage from 0-100%. This method
     * is virtual and must be implemented by derived classes
     * @param force The force to apply in Newton (N)
     * @return double The percentage of the total force a vehicle can apply, from 0-100%
     */
    virtual double force_to_percentage(double force);

    /**
     * @brief Method that is used to convert a percentage from 0-100% of total force a vehicle can apply and 
     * convert it to a force in Newton (N). This method is virtual and must implemented by derived classes
     * @param percentage The percentage of the total force a vehicle can apply, from 0-100%
     * @return double The force to apply in Newton (N)
     */
    virtual double percentage_to_force(double percentage);

    /**
     * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
     * @return double The maximum force the vehicle can apply in Newton (N)
     */
    virtual double get_max_force();

    /**
     * @brief Get the type of thrust curve that is instantiated
     * @return std::string A string with the name of the thrust curve instantiated
     */
    virtual std::string get_type() { return IDENTIFIER; };

    /**
     * @brief Destructor of the LinearExponentialThrustCurve base class. This is empty
     * as no memory allocation is made
     */
    ~LinearExponentialThrustCurve() {};
    
private:
    
    /**
     * @brief Construct a new Quadratic Thrust Curve object, that will encode a thrust curve of the type
     * force = ax^2+bx+c, with x a percentage between 0-100%
     * @param a The linear-exponential gain
     * @param b The exponential gain
     * @param c The linear gain
     * @param d The offset
     * @param scale The scale through x is affected. By default is 1.0. Sometimes, the parameters a, b, and c
     * are defined assuming that the percentage varies from [0, 1], hence if in thoses cases we choose scale = 100, then
     * the output percentage will be scaled such that it varies from [0, 100%]
     */
    LinearExponentialThrustCurve(double a, double b, double c, double d, double scale=1.0);

    /**
     * @brief Quadratic equation terms
     */
    double a_, b_, c_, d_, scale_;

    /**
     * @brief The max force that the vehicle can output in Newton (N)
     */
    double max_force_;

    /**
     * @brief The unique identifier for this thrust curve
     */
    static const std::string IDENTIFIER;

    /**
     * @brief A Bolean that ensures that the thrust
     * curve is registered with the factory at compile time
     */
    static const bool REGISTERED_WITH_FACTORY;
};

}
