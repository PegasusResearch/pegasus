#pragma once

#include <memory>
#include "speed.hpp"

namespace Pegasus::Paths {

class ConstSpeed : public Speed {

public:

    using SharedPtr = std::shared_ptr<ConstSpeed>;
    using UniquePtr = std::unique_ptr<ConstSpeed>;
    using WeakPtr = std::weak_ptr<ConstSpeed>;

    /**
     * @brief Constructuctor for a const speed object
     * @param vehicle_speed A constant vehicle speed expressed in m/s (default=1.0m/s)
     */
    ConstSpeed(double vehicle_speed=1.0) : vehicle_speed_(vehicle_speed) {}

    /**
     * @brief Get the desired speed progression for the path parametric value
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired speed progression for the path parametric value
     */
    virtual double get_vd(double gamma, Section & section);

    /**
     * @brief Get the desired acceleration progression for the path parametric value
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired acceleration progression for the path parametric value
     */
    virtual double get_d_vd(double gamma, Section & section);

    /**
     * @brief Get the vehicle speed progression (in m/s)
     * @param gamma The path parametric value
     * @param section A reference to a section the speed is associated with
     * @return double The desired acceleration progression for the path parametric value
     */
    virtual double get_vehicle_speed(double gamma, Section & section);

    /**
     * @brief Destructor of the Speed base class. This destructor is virtual
     * such that inherited classes do not invoke this destructor when invoking
     * the delete keyword using a BaseClass pointer
     */
    ~ConstSpeed() {}

protected:

    /**
     * @brief The desired vehicle speed progression in m/s
     */
    double vehicle_speed_;

};
}