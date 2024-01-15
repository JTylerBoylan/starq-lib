#ifndef STARQ__GAIT_CONTROLLER_HPP_
#define STARQ__GAIT_CONTROLLER_HPP_

#include <memory>
#include "eigen3/Eigen/Dense"

namespace starq
{
    using namespace Eigen;

    class GaitController
    {
    public:
        using Ptr = std::shared_ptr<GaitController>;

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        virtual std::string getGaitName() = 0;

        /// @brief Start the gait.
        virtual void start() = 0;

        /// @brief Stop the gait.
        virtual void stop() = 0;

        /// @brief Set the velocity of the gait.
        /// @param linear_speed Linear velocity of the gait.
        /// @param angular_speed Angular velocity of the gait.
        virtual void setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed) = 0;

    private:
    };

}

#endif