#include "starq/dynamics/starq_fivebar2d.hpp"

namespace starq::dynamics
{

        STARQ_FiveBar2D::STARQ_FiveBar2D(float L1, float L2, float GR1, float GR2)
            : L1_(L1), L2_(L2), GR1_(GR1), GR2_(GR2) {}

        bool STARQ_FiveBar2D::getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position)
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

        bool STARQ_FiveBar2D::getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles)
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

        bool STARQ_FiveBar2D::getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian)
        {
            if (joint_angles.size() != 2)
                return false;

            const float thetaA = joint_angles(0);
            const float thetaB = joint_angles(1);

            /* TODO : Jacobian */

            return true;
        }
}