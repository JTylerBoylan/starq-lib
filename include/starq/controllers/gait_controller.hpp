#ifndef STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_

#include <memory>
#include "eigen3/Eigen/Dense"

namespace starq::controllers
{
    using namespace Eigen;

    class GaitController
    {
    public:
        using Ptr = std::shared_ptr<GaitController>;

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        virtual std::string getGaitName() = 0;

        /// @brief Move to a position.
        /// @param position Position to move to.
        /// @param orientation Orientation to move to.
        virtual void moveTo(const Vector3f &position, const Vector3f &orientation) = 0;

        /// @brief Stop the gait.
        virtual void stop() = 0;

    private:
    };

}

#endif