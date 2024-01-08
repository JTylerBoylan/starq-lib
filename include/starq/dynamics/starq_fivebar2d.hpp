#ifndef STARQ_DYNAMICS__STARQ_FIVEBAR2D_DYNAMICS_HPP_
#define STARQ_DYNAMICS__STARQ_FIVEBAR2D_DYNAMICS_HPP_

#include "starq/dynamics/leg_dynamics.hpp"

namespace starq::dynamics
{

    class STARQ_FiveBar2D : public LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<STARQ_FiveBar2D>;

        STARQ_FiveBar2D(float L1, float L2, float GR1, float GR2);

        /// @brief Forward kinematics for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        bool getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position) override;

        /// @brief Inverse kinematics for FiveBar2D leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) override;

        /// @brief Jacobian for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        bool getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian) override;

    private:
        float L1_, L2_;
        float GR1_, GR2_;
    };

}

#endif