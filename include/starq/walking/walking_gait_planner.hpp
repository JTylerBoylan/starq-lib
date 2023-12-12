#ifndef STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_
#define STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_

#include "starq/slam/localization.hpp"

namespace starq::walking
{

    struct CenterOfMassTrajectory
    {
        std::vector<Eigen::Vector3f> positions;
        std::vector<Eigen::Vector3f> orientations;
        std::vector<Eigen::Vector3f> velocities;
        std::vector<Eigen::Vector3f> angular_velocities;
    };

    class WalkingGaitPlanner
    {
    public:
        using Ptr = std::shared_ptr<WalkingGaitPlanner>;

        /// @brief Create a walking gait planner.
        WalkingGaitPlanner(starq::slam::Localization::Ptr localization);

        /// @brief Destroy the walking gait planner.
        ~WalkingGaitPlanner();

        /// @brief Get the center of mass trajectory.
        /// @param goal_position Goal position.
        /// @param goal_orientation Goal orientation.
        /// @param trajectory Center of mass trajectory.
        /// @return True if the trajectory was generated successfully, false otherwise.
        bool getCenterOfMassTrajectoryToGoal(const Eigen::Vector3f &goal_position,
                                             const Eigen::Vector3f &goal_orientation,
                                             CenterOfMassTrajectory &trajectory);

    private:
        starq::slam::Localization::Ptr localization_;
    };

}

#endif