#pragma once

#include <memory>
#include <Eigen/Core>
#include "speeds/speed.hpp"

namespace Pegasus::Paths {

class Section {
public:

    using SharedPtr = std::shared_ptr<Section>;
    using UniquePtr = std::unique_ptr<Section>;
    using WeakPtr = std::weak_ptr<Section>;

    /**
     * @brief The section parametric equation 
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the equation of the path with respect to the path parameter gamma
     */ 
    virtual Eigen::Vector3d pd(double gamma) = 0;

    /**
     * @brief First derivative of the path section equation with respect to path parameter gamma
     * @param gamma The path parameter
     * @return An Eigen::Vector3d with the first derivative of the path equation with respect to the path parameter
     */
    virtual Eigen::Vector3d d_pd(double gamma) = 0;

    /**
     * @brief Second derivative of the path section equation with respect to the path parameter gamma
     * @param gamma  The path parameter
     * @return An Eigen::Vector3d with the second derivative of the path equation with respect to the path paramter
     */
    virtual Eigen::Vector3d dd_pd(double gamma) = 0;

    /** 
     * @brief Default method for computing the curvature. The default implementation
     * implements the general formula to compute the curvature based on the derivative
     * equations of the path
     * @param gamma The path parameter
     * @return A double with the path curvature 
     */
    virtual double curvature(double gamma);

    /** 
     * @brief Default method for computing the torsion. The default implementation
     * implements the general formula to compute the curvature based on the derivative
     * equations of the path
     * @param gamma The path parameter
     * @return A double with the path torsion 
     */
    virtual double torsion(double gamma);

    /**
     * @brief Default method for computing the tangent angle to the path 
     * @param gamma The path parameter
     * @return A double with the angle of the tangent to the path expressed in radians
     */
    virtual double tangent_angle(double gamma);

    /**
     * @brief Default method for computing the norm of the derivative 
     * @param gamma  The path parameter
     * @return A double with the norm of the derivative of the path position pd
     */
    virtual double derivative_norm(double gamma);

    /**
     * @brief Default method for getting the desired vehicle speed for a particular 
     * location in the path section (in m/s)
     * @param gamma The path parameter
     * @return double The desired vehicle speed (in m/s)
     */
    virtual double vehicle_speed(double gamma);

    /**
     * @brief Default method for getting the desired speed for the evolution
     * of the parametric value
     * @param The path parameter
     * @return double The desired speed progression of the parametric variable
     */
    virtual double vd(double gamma);

    /**
     * @brief Default method for getting the desired acceleration for the evolution
     * of the parametric value
     * @param gamma The path parameter
     * @return double The speed progression of the parametric variable
     */
    virtual double d_vd(double gamma);

    /**
     * @brief Destructor of the Section base class. This destructor is virtual
     * such that inherited classes do not invoke this destructor when invoking
     * the delete keyword using a BaseClass pointer
     */
    virtual ~Section() {}

    /**
     * @brief Method that limits the path parametric value within the section bounds
     * @param gamma The path parameter
     * @return double The saturated path parameter
     */
    inline double limit_gamma(double gamma) {
        return std::max(min_gamma_, std::min(max_gamma_, gamma));
    }

    /**
     * @brief Get a string which describes the section type
     * @return std::string A string with the section type
     */
    virtual inline std::string get_section_type() {
        return section_type_;
    }

    /**
     * @brief Method to get a set of sampled points from the path
     * @param step_size The step size to increment the parametric value gamma for sampling purposes. Smaller
     * gammas yield a finner result but also more points
     * @return std::vector<Eigen::Vector3d> A vector of 3d points
     */
    virtual std::vector<Eigen::Vector3d> get_samples(double step_size);

protected:

    /**
     * @brief Construct a new Section object
     * @param vehicle_speed The vehicle speed progression along the path object (shared pointer)
     * @param section_type A string which describes the type of section implemented
     * @param min_gamma The minimum parametric value that::SharedPtr encodes a valid portion of the path section (default=0.0)
     * @param max_gamma The maximum parametric value that encodes a valid portion of the path section (default=1.0)
     */
    Section(const std::shared_ptr<Speed> vehicle_speed, const std::string & section_type, double min_gamma=0.0, double max_gamma=1.0) : vehicle_speed_(vehicle_speed), section_type_(section_type), min_gamma_(min_gamma), max_gamma_(max_gamma) {}

private:

    /**
     * @brief The desired vehicle speed along the path object
     */
    std::shared_ptr<Speed> vehicle_speed_;

    /**
     * @brief A string which describes the type of section being used
     */
    std::string section_type_{""};

    /**
     * @brief The minimum parametric value that encodes a valid portion of the path section equation
     */
    double min_gamma_{0.0};

    /**
     * @brief The maximum parametric value that encodes a valid portion of the path section equation
     */
    double max_gamma_{1.0};
};

}