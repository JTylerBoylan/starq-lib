#ifndef STARQ_SLAM__LOCALIZATION_HPP_
#define STARQ_SLAM__LOCALIZATION_HPP_

#include <memory>

#include "eigen3/Eigen/Dense"

namespace starq::slam
{
    using namespace Eigen;

    class Localization
    {
    public:
        using Ptr = std::shared_ptr<Localization>;

        /// @brief Create a localization.
        Localization() {}

        /// @brief Destroy the localization.
        ~Localization() {}

        /// @brief Get the position.
        /// @return Position.
        virtual Vector3f getPosition() = 0;

        /// @brief Get the orientation.
        /// @return Orientation.
        virtual Vector3f getOrientation() = 0;

        /// @brief Get the velocity.
        /// @return Velocity.
        virtual Vector3f getVelocity() = 0;

        /// @brief Get the angular velocity.
        /// @return Angular velocity.
        virtual Vector3f getAngularVelocity() = 0;

    private:
    };
}

#endif