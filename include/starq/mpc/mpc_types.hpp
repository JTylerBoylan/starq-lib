#ifndef STARQ_MPC__MPC_TYPES_HPP_
#define STARQ_MPC__MPC_TYPES_HPP_

#include <vector>
#include "eigen3/Eigen/Dense"

#define NUMBER_OF_LEGS 4

namespace starq::mpc
{
    using namespace Eigen;

    struct LegState
    {
        bool in_stance;
        Eigen::Vector3f foothold_position; // In world frame
    };

    using StrideState = std::array<LegState, NUMBER_OF_LEGS>;

    using StrideTrajectory = std::vector<StrideState>;

    struct CenterOfMassState
    {
        Eigen::Vector3f position;
        Eigen::Vector3f orientation;
        Eigen::Vector3f velocity;
        Eigen::Vector3f angular_velocity;
    };

    using CenterOfMassTrajectory = std::vector<CenterOfMassState>;

    using LegForces = std::array<Vector3f, NUMBER_OF_LEGS>;

}

#endif