/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this 
 * software must display the following acknowledgement: This product 
 * includes software developed by Project Pegasus.
 * 4. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived 
 * from this software without specific prior written permission.
 *
 * Additional Restrictions:
 * 4. The Software shall be used for non-commercial purposes only. 
 * This includes, but is not limited to, academic research, personal 
 * projects, and non-profit organizations. Any commercial use of the 
 * Software is strictly prohibited without prior written permission 
 * from the copyright holders.
 * 5. The Software shall not be used, directly or indirectly, for 
 * military purposes, including but not limited to the development 
 * of weapons, military simulations, or any other military applications. 
 * Any military use of the Software is strictly prohibited without 
 * prior written permission from the copyright holders.
 * 6. The Software may be utilized for academic research purposes, 
 * with the condition that proper acknowledgment is given in all 
 * corresponding publications.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#include <cmath>
#include <iostream>
#include "thrust_curves/thrust_curves.hpp"

namespace Pegasus {

/**
 * -------------------------------------------------------------
 * In this section we define all the methods associated with the 
 * thrust curve abstract class
 * -------------------------------------------------------------
 */

/**
 * @brief Destructor of the ThrustCurve base class. This destructor is virtual
 * such that inherited classes do not invoke this destructor when invoking
 * the delete keyword using a BaseClass pointer
 */
ThrustCurve::~ThrustCurve() {}

/**
 * -------------------------------------------------------------
 * In this section we define all the methods associated with the 
 * thrust curve factory class
 * -------------------------------------------------------------
 */

/**
 * @brief Method used to instantiate a ThrustCurve object
 * @return ThrustCurve A shared pointer to a ThrustCurve object
 */
ThrustCurve::SharedPtr ThrustCurveFactory::create_thrust_curve(std::map<std::string, double> gains, std::string identifier) {

    // Check if the thrust curve is registered in the thrust curve factory. If not, throw a runtime exception
    auto it = creators_.find(identifier);

    if(it == creators_.end()) {
        throw std::runtime_error("Thrust curve " + identifier + " not registered in ThrustCurveFactory!");
    }

    // Check the map of the thrust curve factory and call the correct function that will instantiate a 
    // thrust curve of the desired type
    return creators_[identifier](gains);
}

/**
 * @brief Get the singleton instance object of this thrust curve factory (only one object of this type per program)
 * @return ThrustCurveFactory& A reference to the singleton instance of this object
 */
ThrustCurveFactory& ThrustCurveFactory::get_instance() {
    // Create a static object of ThrustCurveFactory inside the get_instace function,
    // such that the first time this function is called, this object is instantated,
    // but for the remaining calls in remains the same
    // NOTE: static in class != static in function
    static ThrustCurveFactory instance = ThrustCurveFactory();
    return instance;
}

/**
 * @brief Assign a thrust function with its creator 
 * @param _identifier A string which identifies the type of thrust curve that will be saved
 * @param _creator A function that receives a map of parameters <string, double> and a string with an identifier
 * as inputs and returns a pointer to an object of the type ThrustCurve
 * @return bool A boolean whether a ThrustCurve object was registed in the ThrustCurveFactory successfully or not
 */
bool ThrustCurveFactory::register_creator(const std::string& _identifier, ThrustCurveCreator _creator) {

    /* Check if the thrust curve object already exists inside the map of thrust curves */
    if(creators_.find(_identifier) != creators_.end()) {
        std::cerr << "Error: Registering a thrust curve " << _identifier << "more than once!" << std::endl;
        return false;
    }

    /* Save the creator function of the thrust curve to the map, along with its name/identifier */
    creators_[_identifier] = _creator;
    return true;
}

/**
 * -------------------------------------------------------------
 * In this section we define all the methods associated with the 
 *                   QuadraticThrustCurve 
 * -------------------------------------------------------------
 */

/**
 * @brief Define the string Idenfifier for this thrust curve
 */
const std::string QuadraticThrustCurve::IDENTIFIER = "Quadratic";

/**
 * @brief Register this thrust curve in the ThrustCurveFactory
 */
const bool QuadraticThrustCurve::REGISTERED_WITH_FACTORY = ThrustCurveFactory::get_instance().register_creator(QuadraticThrustCurve::IDENTIFIER, QuadraticThrustCurve::create_thrust_curve);

/**
 * @brief Static method used to instantiate a quadratic thrust curve object
 * @return ThrustCurve A shared pointer to a ThrustCurve object
 */
ThrustCurve::SharedPtr QuadraticThrustCurve::create_thrust_curve(std::map<std::string, double> gains) {
    return std::shared_ptr<QuadraticThrustCurve>(new QuadraticThrustCurve(gains["a"], gains["b"], gains["c"], gains["scale"]));
}

/**
 * @brief Construct a new Quadratic Thrust Curve object, that will encode a thrust curve of the type
 * force = ax^2+bx+c, with x a percentage between 0-100%
 * @param a The qudratic gain
 * @param b The linear gain
 * @param c The offset
 */
QuadraticThrustCurve::QuadraticThrustCurve(double a, double b, double c, double scale) : a_(a), b_(b), c_(c), scale_(scale) {
    max_force_ = percentage_to_force(100.0);
    parameters_["a"] = a_;
    parameters_["b"] = b_;
    parameters_["c"] = c_;
    parameters_["scale"] = scale_;
}

/**
 * @brief Method that is used to convert a force in Newton (N) to a percentage from 0-100%. This method
 * is virtual and must be implemented by derived classes
 * @param force The force to apply in Newton (N)
 * @return double The percentage of the total force a vehicle can apply, from 0-100%
 */
double QuadraticThrustCurve::force_to_percentage(double force) {
    // Make sure the desired force to apply in Newtons is not bellow zero
    double f = std::max(0.0, force);
    return std::max(0.0, std::min(100.0, (-b_ + std::sqrt(std::pow(b_, 2) - (4 * a_ * (c_ - f)))) / (2 * a_) * scale_));
}

/**
 * @brief Method that is used to convert a percentage from 0-100% of total force a vehicle can apply and 
 * convert it to a force in Newton (N). This method is virtual and must implemented by derived classes
 * @param percentage The percentage of the total force a vehicle can apply, from 0-100%
 * @return double The force to apply in Newton (N)
 */
double QuadraticThrustCurve::percentage_to_force(double percentage) {
    double x = std::min(100.0, std::max(0.0, percentage)) / scale_;
    return (a_ * std::pow(x, 2)) + (b_ * x) + c_;
}

/**
 * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
 * @return double The maximum force the vehicle can apply in Newton (N)
 */
double QuadraticThrustCurve::get_max_force() {
    return max_force_;
}

/**
 * -------------------------------------------------------------
 * In this section we define all the methods associated with the 
 *                   ArctangentThrustCurve 
 * ------------------------------------------------------------- 
 */

/**
 * @brief Define the string Idenfifier for this thrust curve
 */
const std::string ArctangentThrustCurve::IDENTIFIER = "Arctan";

/**
 * @brief Register this thrust curve in the ThrustCurveFactory
 */
const bool ArctangentThrustCurve::REGISTERED_WITH_FACTORY = ThrustCurveFactory::get_instance().register_creator(ArctangentThrustCurve::IDENTIFIER, ArctangentThrustCurve::create_thrust_curve);

/**
 * @brief Static method used to instantiate a Arctangent thrust curve object
 * @return ThrustCurve A shared pointer to a ThrustCurve object
 */
ThrustCurve::SharedPtr ArctangentThrustCurve::create_thrust_curve(std::map<std::string, double> gains) {
    return std::shared_ptr<ArctangentThrustCurve>(new ArctangentThrustCurve(gains["a"], gains["b"], gains["c"], gains["d"]));
}

ArctangentThrustCurve::ArctangentThrustCurve(double a, double b, double c, double d) : a_(a), b_(b), c_(c), d_(d) {

    max_force_ = percentage_to_force(100.0);
    parameters_["a"] = a;
    parameters_["b"] = b;
    parameters_["c"] = c;
    parameters_["d"] = d;
}

double ArctangentThrustCurve::force_to_percentage(double force) {

    // Make sure the desired force is within the limits
    double f = std::min(std::max(0.0, force), max_force_);

    // Compute the output in percentage
    double u = (std::tan((f-d_)/a_)-c_)/b_;

    // Output the percentage clipped between 0-100%. It should be, but just in case
    return std::max(0.0, std::min(u, 100.0));
}

double ArctangentThrustCurve::percentage_to_force(double percentage) {

    // Clip the percentage between 0-100%
    double x = std::min(100.0, std::max(0.0, percentage));

    // Return the force in Newtons
    return (a_ * std::atan(b_ * x + c_)) + d_;
}

/**
 * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
 * @return double The maximum force the vehicle can apply in Newton (N)
 */
double ArctangentThrustCurve::get_max_force() {
    return max_force_;
}


/**
 * -------------------------------------------------------------
 * In this section we define all the methods associated with the 
 *                   LinearExponentialThrustCurve 
 * -------------------------------------------------------------
 */

/**
 * @brief Define the string Idenfifier for this thrust curve
 */
const std::string LinearExponentialThrustCurve::IDENTIFIER = "LinearExponential";

/**
 * @brief Register this thrust curve in the ThrustCurveFactory
 */
const bool LinearExponentialThrustCurve::REGISTERED_WITH_FACTORY = ThrustCurveFactory::get_instance().register_creator(LinearExponentialThrustCurve::IDENTIFIER, LinearExponentialThrustCurve::create_thrust_curve);

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
LinearExponentialThrustCurve::LinearExponentialThrustCurve(double a, double b, double c, double d, double scale) : a_(a), b_(b), c_(c), d_(d), scale_(scale) {
    max_force_ = percentage_to_force(100.0);
    parameters_["a"] = a_;
    parameters_["b"] = b_;
    parameters_["c"] = c_;
    parameters_["d"] = d_;
    parameters_["scale"] = scale_;
}

/**
 * @brief Static method used to instantiate a quadratic thrust curve object
 * @param gains The quadratic gains that define the curve. A map with string {a: double, b: double, c: double, d: double, scale:double}
 * that defines the equation 
 * percentage = (a * exp(b*force) * sqrt(force) + (c * force) + d) * scale
 * @return ThrustCurve A shared pointer to a ThrustCurve object
 */
ThrustCurve::SharedPtr LinearExponentialThrustCurve::create_thrust_curve(std::map<std::string, double> gains) {
    return std::shared_ptr<LinearExponentialThrustCurve>(new LinearExponentialThrustCurve(gains["a"], gains["b"], gains["c"], gains["d"], gains["scale"]));
}

/**
 * @brief Method that is used to convert a force in Newton (N) to a percentage from 0-100%. This method
 * is virtual and must be implemented by derived classes
 * @param force The force to apply in Newton (N)
 * @return double The percentage of the total force a vehicle can apply, from 0-100%
 */
double LinearExponentialThrustCurve::force_to_percentage(double force) {
    // Make sure the desired force to apply in Newtons is not bellow zero
    double f = std::max(0.0, force);
    return std::min(100.0, std::max(0.0, ((a_ * std::exp(b_ * f) * std::sqrt(f)) + (c_ * f) + d_) * scale_));
}

/**
 * @brief Method that is used to convert a percentage from 0-100% of total force a vehicle can apply and 
 * convert it to a force in Newton (N). This method is virtual and must implemented by derived classes
 * @param percentage The percentage of the total force a vehicle can apply, from 0-100%
 * @return double The force to apply in Newton (N)
 */
double LinearExponentialThrustCurve::percentage_to_force(double percentage) {
    (void) percentage;
    return 0.0; // TODO - try to invert this expression the best I can :)
}

/**
 * @brief Get the max force that the vehicle can output when given 100% of percentage of thrust
 * @return double The maximum force the vehicle can apply in Newton (N)
 */
double LinearExponentialThrustCurve::get_max_force() {
    return max_force_;
}
    
}