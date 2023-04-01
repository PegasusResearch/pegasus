#pragma once

#include <memory>

#include "camera_models/camera.hpp"

namespace Pegasus {

class FeatureTracker {

public:

     /**
     * @brief An alias for a shared pointer of a FeatureTracker
     */
    using SharedPtr = std::shared_ptr<FeatureTracker>;

    /**
     * @brief An alias for a unique pointer of a FeatureTracker
     */
    using UniquePtr = std::unique_ptr<FeatureTracker>;

    /**
     * @brief An alias for a weak pointer of a FeatureTracker
     */
    using WeakPtr = std::weak_ptr<FeatureTracker>;

    /**
     * @brief Default constructor
     */
    FeatureTracker();

    /**
     * @brief Default destructor
     */
    ~FeatureTracker();


protected:

    /**
     * @brief The camera model
     */
    Camera camera_;

    
};

}