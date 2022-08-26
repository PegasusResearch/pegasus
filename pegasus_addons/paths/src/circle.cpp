#include "paths/circle.hpp"

namespace Pegasus::Paths {

/**
 * @brief Constructor for a new Circle path section
 * @param vehicle_speed A shared pointer to a desired vehicle speed
 * @param center A 3D vector with the starting point for the line
 * @param normal The normal vector that defines the plane where the 2D circle will be placed
 * @param radius The radius of the circle in meters (m)
 */
Circle::Circle(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & center, const Eigen::Vector3d & normal, const double radius) : 
    Section(vehicle_speed, 0.0, 1.0), center_(center), normal_(normal), radius_(radius), curvature_(1.0 / radius) { 

    // ------------------------
    // Initialize the rotation matrix with the rotation 
    // information encoded in the normal vector
    // -----------------------
    
    // Step 1 - Check that the normal vector != [0, 0, 1] and [0,0, -1] otherwize we would not need to rotate
    // and the rotation matrix would be hill-posed using the following method
    Eigen::Vector3d base_normal(0.0, 0.0, 1.0);
    Eigen::Vector3d absolute_normal = normal_.array().abs();

    // Because we are using doubles, we must check if the vectors are not approximatelly [0, 0, 1] 
    if ((absolute_normal - base_normal).norm() > 0.0001) {

        // Step 2 - Now that we know that the vector is valid, then compute the rotation matrix
        Eigen::Vector3d u3 = normal_.normalized();
        Eigen::Vector3d u1 = (u3.cross3(base_normal)).normalized();
        Eigen::Vector3d u2 = (u3.cross3(u1)).normalized();

        // Step 3 - assign the normalized vectors to the columns of the rotation matrix
        rotation_.col(0) = u1;
        rotation_.col(1) = u2;
        rotation_.col(2) = u3;
    }
}

/**
 * @brief Constructor for a new Circle path section. This circle will be inscribed in a xy-plane
 * centered at "center"
 * @param vehicle_speed A shared pointer to a desired vehicle speed
 * @param center A 3D vector with the starting point for the line
 * @param radius The radius of the circle in meters (m)
 */
Circle::Circle(const std::shared_ptr<Speed> vehicle_speed, const Eigen::Vector3d & center, const double radius) : 
    Section(vehicle_speed, 0.0, 1.0), center_(center), radius_(radius), curvature_(1.0 / radius) { }

/**
 * @brief The section parametric equation 
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
 */
Eigen::Vector3d Circle::pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the location of the 2D circle in a plane centered around [x,y, 0.0]
    Eigen::Vector3d pd;
    pd[0] = radius_ * cos(t * 2 * M_PI);
    pd[1] = radius_ * sin(t * 2 * M_PI);
    pd[2] = 0.0;
    
    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    pd = rotation_ * pd;

    // Add theoffset to the circle after the rotation, otherwise the offset would also get rotated
    return pd + center_;
}

/**
 * @brief First derivative of the path section equation with respect to path parameter gamma
 * @param gamma The path parameter
 * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
 */
Eigen::Vector3d Circle::d_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the first derivative of the circle with respect to the parametric value
    Eigen::Vector3d d_pd;
    d_pd[0] = -radius_ * 2 * M_PI * sin(t * 2 * M_PI);
    d_pd[1] =  radius_ * 2 * M_PI * cos(t * 2 * M_PI);
    d_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    return rotation_ * d_pd;
}

/**
 * @brief Second derivative of the path section equation with respect to the path parameter gamma
 * @param gamma  The path parameter
 * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
 */
Eigen::Vector3d Circle::dd_pd(double gamma) {
    
    // Bound the parametric value
    double t = limit_gamma(gamma);

    // Compute the second derivative of the circle with respect to the parametric value
    Eigen::Vector3d dd_pd;
    dd_pd[0] = -radius_ * std::pow(2 * M_PI, 2) * cos(t * 2 * M_PI);
    dd_pd[1] = -radius_ * std::pow(2 * M_PI, 2) * sin(t * 2 * M_PI);
    dd_pd[2] = 0.0;

    // If the "normal_" vector is different than [0.0, 0.0, 1.0], then 
    // rotate the plane where the circle is located. Otherwise, we are just multiplying by the identity matrix
    return rotation_ * dd_pd;
}

/** 
 * @brief Override and just returns 0.0
 * @param gamma The path parameter
 * @return A double with the line curvature  = 0
 */
double Circle::curvature(double gamma) {
    return curvature_;
}

}