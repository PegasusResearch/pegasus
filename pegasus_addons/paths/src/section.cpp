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
}