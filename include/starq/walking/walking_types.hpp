#ifndef STARQ_WALKING__WALKING_TYPES_HPP_
#define STARQ_WALKING__WALKING_TYPES_HPP_

#include "eigen3/Eigen/Dense"

namespace starq::walking
{

    struct LegState
    {
        bool in_stance;
        Eigen::Vector3f position;
    };

    using StrideTrajectory = std::vector<LegState>;

    struct CenterOfMassState
    {
        Eigen::Vector3f position;
        Eigen::Vector3f orientation;
        Eigen::Vector3f velocity;
        Eigen::Vector3f angular_velocity;
    };

    using CenterOfMassTrajectory = std::vector<CenterOfMassState>;

}

#endif