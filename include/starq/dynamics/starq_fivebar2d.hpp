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

            const float thetaA = joint_angles(0) / GR1_;
            const float thetaB = joint_angles(1) / GR2_;

            const float alpha = 0.5f * (M_PI - thetaA - thetaB);
            const float gamma = std::asin(L1_ * std::sin(alpha) / L2_);
            const float phi = M_PI - alpha - gamma;

            const float theta = thetaA + alpha;
            const float R = L2_ * std::sin(phi) / std::sin(alpha);

            const float X = R * std::cos(theta);
            const float Y = R * std::sin(theta);

            foot_position = Vector2f(X, Y);

            return true;
        }

        /// @brief Inverse kinematics for FiveBar2D leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool inverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) override
        {

            if (foot_position.size() != 2)
                return false;

            const float X = foot_position(0);
            const float Y = foot_position(1);

            const float theta0 = std::atan2(Y, X);
            const float theta1 = std::atan2(Y, -X);
            const float R = std::sqrt(X * X + Y * Y);
            const float alpha = std::acos((R * R + L1_ * L1_ - L2_ * L2_) / (2.0f * R * L1_));

            const float thetaA = GR1_ * (theta0 + alpha) / (2.0f * M_PI);
            const float thetaB = GR2_ * (theta1 + alpha) / (2.0f * M_PI);

            joint_angles = Vector2f(thetaA, thetaB);

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

            /* TODO : Jacobian */

            return false;
        }

        /// @brief Set leg lengths.
        /// @param L1 Length of link 1.
        /// @param L2 Length of link 2.
        void setLegLengths(float L1, float L2)
        {
            L1_ = L1;
            L2_ = L2;
        }

        /// @brief Set gear ratios.
        /// @param GR1 Gear ratio of joint 1.
        /// @param GR2 Gear ratio of joint 2.
        void setGearRatios(float GR1, float GR2)
        {
            GR1_ = GR1;
            GR2_ = GR2;
        }

    private:
        float L1_, L2_;
        float GR1_, GR2_;
    };

}

#endif