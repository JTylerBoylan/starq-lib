#ifndef STARQ_DYNAMICS__STARQ_RRR_DYNAMICS_HPP_
#define STARQ_DYNAMICS__STARQ_RRR_DYNAMICS_HPP_

#include "starq/dynamics/leg_dynamics.hpp"

namespace starq::dynamics
{
    class STARQ_RRR : public LegDynamics
    {
        using Ptr = std::shared_ptr<STARQ_RRR>;

        /// @brief Constructor for RRR leg.
        /// @param L1 First leg length.
        /// @param L2 Second leg length.
        /// @param L3 Third leg length.
        /// @param GR1 First gear ratio.
        /// @param GR2 Second gear ratio.
        /// @param GR3 Third gear ratio.
        STARQ_RRR(float L1, float L2, float L3, float GR1, float GR2, float GR3)
            : L1_(L1), L2_(L2), L3_(L3), GR1_(GR1), GR2_(GR2), GR3_(GR3)
        {
        }

        /// @brief Forward kinematics for RRR leg.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        bool forwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position) override
        {
            if (joint_angles.size() != 3)
                return false;

            const float thetaA = joint_angles(0) / GR1_;
            const float thetaB = joint_angles(1) / GR2_;
            const float thetaC = joint_angles(2) / GR3_;

            const float cosA = std::cos(thetaA);
            const float cosB = std::cos(thetaB);
            const float cosC = std::cos(thetaC);
            const float sinA = std::sin(thetaA);
            const float sinB = std::sin(thetaB);
            const float sinC = std::sin(thetaC);

            const float xe = L3_ * (cosB * cosC - sinB * sinC) + L2_ * cosB;
            const float ye = L1_ * cosA - L3_ * (sinA * cosB * sinC + sinA * sinB * cosC) - L2_ * sinA * sinB;
            const float ze = L1_ * sinA + L3_ * (cosA * cosB * sinC + cosA * sinB * cosC) + L2_ * cosA * sinB;

            foot_position = Vector3f(xe, ye, ze);

            return true;
        }

        /// @brief Inverse kinematics for RRR leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool inverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) override
        {
            (void)foot_position; // prevent unused parameter warning
            (void)joint_angles;

            /* TODO : Implement inverse kinematics. */

            return false;
        }

        /// @brief Jacobian for RRR leg.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        bool jacobian(const VectorXf &joint_angles, MatrixXf &jacobian) override
        {
            if (joint_angles.size() != 3)
                return false;

            const float thetaA = joint_angles(0);
            const float thetaB = joint_angles(1);
            const float thetaC = joint_angles(2);

            const float cosA = std::cos(thetaA);
            const float cosB = std::cos(thetaB);
            const float cosC = std::cos(thetaC);
            const float sinA = std::sin(thetaA);
            const float sinB = std::sin(thetaB);
            const float sinC = std::sin(thetaC);

            const float dXdA = 0;
            const float dXdB = -L3_ * (cosB * sinC + cosC * sinB) - L2_ * sinB;
            const float dXdC = -L3_ * (cosB * sinC + cosC * sinB);

            const float dYdA = -L1_ * sinA - L3_ * (cosA * cosB * sinC + cosA * sinB * cosC) - L2_ * cosA * sinB;
            const float dYdB = L3_ * (sinA * sinB * sinC - sinA * cosB * cosC) - L2_ * sinA * cosB;
            const float dYdC = L3_ * (sinA * sinB * sinC - sinA * cosB * cosC);

            const float dZdA = L1_ * cosA - L3_ * (sinA * cosB * sinC + sinA * sinB * cosC) - L2_ * sinA * sinB;
            const float dZdB = -L3_ * (cosA * sinB * sinC - cosA * cosB * cosC) + L2_ * cosA * cosB;
            const float dZdC = -L3_ * (cosA * sinB * sinC - cosA * cosB * cosC);

            jacobian = Matrix3f(dXdA, dXdB, dXdC,
                                dYdA, dYdB, dYdC,
                                dZdA, dZdB, dZdC);

            return true;
        }

        /// @brief Set the leg lengths.
        /// @param L1 First leg length.
        /// @param L2 Second leg length.
        /// @param L3 Third leg length.
        void setLegLengths(float L1, float L2, float L3)
        {
            L1_ = L1;
            L2_ = L2;
            L3_ = L3;
        }

        /// @brief Set the gear ratios.
        /// @param GR1 First gear ratio.
        /// @param GR2 Second gear ratio.
        /// @param GR3 Third gear ratio.
        void setGearRatios(float GR1, float GR2, float GR3)
        {
            GR1_ = GR1;
            GR2_ = GR2;
            GR3_ = GR3;
        }

    private:
        float L1_, L2_, L3_;
        float GR1_, GR2_, GR3_;
    };
}

#endif