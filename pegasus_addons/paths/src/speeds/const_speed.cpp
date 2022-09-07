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