#ifndef STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_

#include "starq/publishers/leg_command_publisher.hpp"

namespace starq::controllers
{

    class GaitController
    {
    public:
        using Ptr = std::shared_ptr<GaitController>;

        /// @brief Create a gait controller.
        /// @param leg_command_publisher Leg command publisher.
        GaitController(const starq::publishers::LegCommandPublisher::Ptr leg_command_publisher)
            : leg_command_publisher_(leg_command_publisher) {}

        /// @brief Destroy the gait controller.
        ~GaitController() {}

        /// @brief Get the leg command publisher.
        /// @return Leg command publisher.
        starq::publishers::LegCommandPublisher::Ptr getLegCommandPublisher() { return leg_command_publisher_; }

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        virtual std::string getGaitName() = 0;

        /// @brief Move to a position.
        /// @param position Position to move to.
        virtual void moveTo(Vector3f position) = 0;

    private:
        starq::publishers::LegCommandPublisher::Ptr leg_command_publisher_;
    };

}

#endif