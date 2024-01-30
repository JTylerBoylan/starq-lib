#ifndef STARQ_MPC__MPC_TYPES_HPP_
#define STARQ_MPC__MPC_TYPES_HPP_

#include <vector>
#include <map>
#include <chrono>

#include "eigen3/Eigen/Dense"

namespace starq::mpc
{
    using namespace Eigen;

    using StanceState = std::vector<bool>;
    using GaitSequence = std::map<time_t, StanceState>;    

    struct CenterOfMassState
    {
        Eigen::Vector3f position;
        Eigen::Vector3f orientation;
        Eigen::Vector3f velocity;
        Eigen::Vector3f angular_velocity;
    };

    using CenterOfMassTrajectory = std::vector<CenterOfMassState>;

    using LegForces = std::vector<Vector3f>;

}

#endif