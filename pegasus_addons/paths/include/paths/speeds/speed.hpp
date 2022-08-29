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
    virtual ~Speed() = 0;

protected:

    /**
     * @brief Empty Constructor for a base Speed. This method cannot be invoked
     * externally, which means it can only be invoked by the constructor of other derived classes
     */
    Speed() {};
};

}