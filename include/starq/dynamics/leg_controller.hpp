#ifndef STARQ_DYNAMICS__LEG_CONTROLLER_HPP_
#define STARQ_DYNAMICS__LEG_CONTROLLER_HPP_

#include <vector>

#include "starq/odrive/controller.hpp"
#include "starq/dynamics/leg_dynamics.hpp"

#define MAX_LEG_ID 0x3F

namespace starq::dynamics
{

    class LegController
    {
    public:
        using Ptr = std::shared_ptr<LegController>;

        /// @brief Create a leg controller.
        /// @param controller ODrive controller.
        LegController(const starq::odrive::ODriveController::Ptr odrive_controller)
            : odrive_controller_(odrive_controller)
        {
        }

        /// @brief Destroy the leg controller.
        ~LegController()
        {
        }

        /// @brief Set the motor CAN IDs for a leg.
        /// @param leg_id ID of the leg.
        /// @param motor_can_ids CAN IDs of the motors.
        void setMotorCANIDs(const uint8_t leg_id, const std::vector<uint32_t> motor_can_ids)
        {
            configs_[leg_id].motor_can_ids = motor_can_ids;
        }

        /// @brief Set the leg dynamics for a leg.
        /// @param leg_id ID of the leg.
        /// @param dynamics Leg dynamics.
        void setLegDynamics(const uint8_t leg_id, const LegDynamics::Ptr dynamics)
        {
            configs_[leg_id].dynamics = dynamics;
        }

        /// @brief Set the axis state for a leg.
        /// @param leg_id ID of the leg.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t leg_id, const uint32_t state)
        {
            bool success = true;
            for (const auto &motor_can_id : configs_[leg_id].motor_can_ids)
            {
                success &= odrive_controller_->setAxisState(motor_can_id, state);
            }
            return success;
        }

        /// @brief Set the control mode for a leg.
        /// @param leg_id ID of the leg.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t leg_id, const uint32_t control_mode, const uint32_t input_mode = 0x1)
        {
            bool success = true;
            for (const auto &motor_can_id : configs_[leg_id].motor_can_ids)
            {
                success &= odrive_controller_->setControlMode(motor_can_id, control_mode, input_mode);
            }
            return success;
        }

        /// @brief Set the position gain for a leg.
        /// @param leg_id ID of the leg.
        /// @param pos_gain Position gain.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t leg_id, const float pos_gain)
        {
            bool success = true;
            for (const auto &motor_can_id : configs_[leg_id].motor_can_ids)
            {
                success &= odrive_controller_->setPosGain(motor_can_id, pos_gain);
            }
            return success;
        }

        /// @brief Set the velocity gain for a leg.
        /// @param leg_id ID of the leg.
        /// @param vel_gain Velocity gain.
        /// @param vel_integrator_gain Velocity integrator gain.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t leg_id, const float vel_gain, const float vel_integrator_gain)
        {
            bool success = true;
            for (const auto &motor_can_id : configs_[leg_id].motor_can_ids)
            {
                success &= odrive_controller_->setVelGains(motor_can_id, vel_gain, vel_integrator_gain);
            }
            return success;
        }

        /// @brief Set the limits for a leg.
        /// @param leg_id ID of the leg.
        /// @param vel_limit Velocity limit.
        /// @param current_limit Current limit.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t leg_id, const float vel_limit, const float current_limit)
        {
            bool success = true;
            for (const auto &motor_can_id : configs_[leg_id].motor_can_ids)
            {
                success &= odrive_controller_->setLimits(motor_can_id, vel_limit, current_limit);
            }
            return success;
        }

        /// @brief Set the foot position for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_position Foot position.
        /// @param foot_velocity_ff Foot velocity feedforward.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return If the command was sent successfully.
        bool setFootPosition(const uint8_t leg_id,
                             const VectorXf &foot_position,
                             const VectorXf &foot_velocity_ff,
                             const VectorXf &foot_torque_ff)
        {
            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Jacobian failed." << std::endl;
                return false;
            }

            VectorXf joint_angles;
            if (!configs_[leg_id].dynamics->inverseKinematics(foot_position, joint_angles))
            {
                std::cerr << "Inverse kinematics failed." << std::endl;
                return false;
            }
            
            VectorXf joint_velocity = jacobian.inverse() * foot_velocity_ff;
            VectorXf joint_torque = jacobian.transpose() * foot_torque_ff;

            return setJointAngles(leg_id, joint_angles, joint_velocity, joint_torque);
        }

    private:
        MatrixXf getCurrentJointAngles(const uint8_t leg_id)
        {
            MatrixXf joint_angles(configs_[leg_id].motor_can_ids.size(), 1);
            for (size_t i = 0; i < configs_[leg_id].motor_can_ids.size(); i++)
            {
                joint_angles(i, 0) = odrive_controller_->getPositionEstimate(configs_[leg_id].motor_can_ids[i]);
            }
            return joint_angles;
        }

        bool setJointAngles(const uint8_t leg_id,
                            const VectorXf &joint_angles,
                            const VectorXf &joint_velocity_ff,
                            const VectorXf &joint_torque_ff)
        {
            bool success = true;
            for (size_t i = 0; i < configs_[leg_id].motor_can_ids.size(); i++)
            {
                success &= odrive_controller_->setPosition(configs_[leg_id].motor_can_ids[i],
                                                           joint_angles(i, 0),
                                                           joint_velocity_ff(i, 0),
                                                           joint_torque_ff(i, 0));
            }
            return success;
        }

        const starq::odrive::ODriveController::Ptr odrive_controller_;

        struct
        {
            std::vector<uint32_t> motor_can_ids;
            LegDynamics::Ptr dynamics;
        } configs_[MAX_LEG_ID + 1];
    };

}

#endif