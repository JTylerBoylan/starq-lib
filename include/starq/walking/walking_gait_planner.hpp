#ifndef STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_
#define STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_

#include "starq/slam/localization.hpp"

namespace starq::walking
{

    struct CenterOfMassTrajectory
    {
        size_t size;
        std::vector<Eigen::Vector3f> positions;
        std::vector<Eigen::Vector3f> orientations;
        std::vector<Eigen::Vector3f> velocities;
        std::vector<Eigen::Vector3f> angular_velocities;
    };

    struct FootfallState
    {
        std::vector<bool> in_stance;
        std::vector<Eigen::Vector3f> positions;
    };

    struct FootfallPattern
    {
        size_t size;
        std::vector<FootfallState> states;
    };

    class WalkingGaitPlanner
    {
    public:
        using Ptr = std::shared_ptr<WalkingGaitPlanner>;

        /// @brief Create a walking gait planner.
        WalkingGaitPlanner(starq::slam::Localization::Ptr localization);

        /// @brief Destroy the walking gait planner.
        ~WalkingGaitPlanner();

        /// @brief Set the desired state.
        /// @param position Desired position.
        /// @param orientation Desired orientation.
        void setDesiredState(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);

        /// @brief Get the center of mass trajectory.
        /// @param trajectory Center of mass trajectory.
        /// @return True if the trajectory was generated successfully, false otherwise.
        bool getCenterOfMassTrajectory(CenterOfMassTrajectory &trajectory);

        /// @brief Get the footfall pattern.
        /// @param pattern Footfall pattern.
        /// @return True if the pattern was generated successfully, false otherwise.
        bool getFootfallPattern(FootfallPattern &pattern);

    private:
        starq::slam::Localization::Ptr localization_;

        Eigen::Vector3f desired_position_;
        Eigen::Vector3f desired_orientation_;
    };

}

#endif