#ifndef STARQ_WALKING__WALKING_GAIT_CONTROLLER_HPP_
#define STARQ_WALKING__WALKING_GAIT_CONTROLLER_HPP_

#include "starq/gait_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/slam/localization.hpp"
#include "starq/walking/walking_gait_planner.hpp"
#include "starq/mpc/body_control_mpc.hpp"

namespace starq::walking
{

    class WalkingGaitController : public starq::GaitController
    {
    public:
        using Ptr = std::shared_ptr<WalkingGaitController>;

        /// @brief Create a walking gait controller.
        WalkingGaitController(starq::LegCommandPublisher::Ptr leg_command_publisher,
                              starq::slam::Localization::Ptr localization);

        /// @brief Destroy the walking gait controller.
        ~WalkingGaitController();

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        std::string getGaitName() override { return "walk"; }

        /// @brief Start the gait.
        void start() override;

        /// @brief Stop the gait.
        void stop() override;

        /// @brief Set the walking speed.
        /// @param linear_speed Linear velocity of the walk.
        /// @param angular_speed Angular velocity of the walk.
        void setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed) override;

    private:

        starq::LegCommandPublisher::Ptr leg_command_publisher_;
        starq::slam::Localization::Ptr localization_;
        starq::mpc::BodyControlMPC::Ptr body_control_mpc_;
        starq::walking::WalkingGaitPlanner::Ptr walking_gait_planner_;
    };

}

#endif