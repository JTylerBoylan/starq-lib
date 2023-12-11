#ifndef STARQ_DYNAMICS__STARQ_FIVEBAR2D_DYNAMICS_HPP_
#define STARQ_DYNAMICS__STARQ_FIVEBAR2D_DYNAMICS_HPP_

#include "starq/dynamics/leg_dynamics.hpp"

namespace starq::dynamics
{

    class STARQ_FiveBar2D : public LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<STARQ_FiveBar2D>;

        STARQ_FiveBar2D(float L1, float L2) : L1_(L1), L2_(L2) {}

        /// @brief Forward kinematics for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        bool forwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position) override
        {
            if (joint_angles.size() != 2)
                return false;

            const float thetaA = joint_angles(0);
            const float thetaB = joint_angles(1);

            const float cosA = std::cos(thetaA);
            const float cosB = std::cos(thetaB);
            const float sinA = std::sin(thetaA);
            const float sinB = std::sin(thetaB);

            /* TODO */

            return true;
        }

        /// @brief Inverse kinematics for FiveBar2D leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool inverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) override
        {

            /* TODO */

            return true;
        }

        /// @brief Jacobian for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        bool jacobian(const VectorXf &joint_angles, MatrixXf &jacobian) override
        {
            if (joint_angles.size() != 2)
                return false;

            const float thetaA = joint_angles(0);
            const float thetaB = joint_angles(1);

            const float cosA = std::cos(thetaA);
            const float cosB = std::cos(thetaB);
            const float sinA = std::sin(thetaA);
            const float sinB = std::sin(thetaB);

            /* TODO */

            return true;
        }

    private:
        float L1_, L2_;
    };

}

#endif