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

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

            configs_[leg_id].motor_can_ids = motor_can_ids;
        }

        /// @brief Set the leg dynamics for a leg.
        /// @param leg_id ID of the leg.
        /// @param dynamics Leg dynamics.
        void setLegDynamics(const uint8_t leg_id, const LegDynamics::Ptr dynamics)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

            configs_[leg_id].dynamics = dynamics;
        }

        /// @brief Set the axis state for a leg.
        /// @param leg_id ID of the leg.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t leg_id, const uint32_t state)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

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

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

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

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

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

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

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

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

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
                             const VectorXf &foot_velocity_ff = VectorXf(),
                             const VectorXf &foot_torque_ff = VectorXf())
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }

            VectorXf joint_angles;
            if (!configs_[leg_id].dynamics->inverseKinematics(foot_position, joint_angles))
            {
                std::cerr << "Failed to get inverse kinematics." << std::endl;
                return false;
            }

            VectorXf joint_velocity = foot_velocity_ff.size() > 0 ? jacobian.inverse() * foot_velocity_ff : VectorXf();
            VectorXf joint_torque = foot_torque_ff.size() > 0 ? jacobian.transpose() * foot_torque_ff : VectorXf();

            return setJointAngles(leg_id, joint_angles, joint_velocity, joint_torque);
        }

        /// @brief Set the foot velocity for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_velocity Foot velocity.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return  If the command was sent successfully.
        bool setFootVelocity(const uint8_t leg_id,
                             const VectorXf &foot_velocity,
                             const VectorXf &foot_torque_ff = VectorXf())
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }

            VectorXf joint_velocity = jacobian.inverse() * foot_velocity;
            VectorXf joint_torque = foot_torque_ff.size() > 0 ? jacobian.transpose() * foot_torque_ff : VectorXf();

            return setJointVelocities(leg_id, joint_velocity, joint_torque);
        }

        /// @brief Set the foot force for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_force Foot force.
        /// @return If the command was sent successfully.
        bool setFootForce(const uint8_t leg_id,
                          const VectorXf &foot_force)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }

            VectorXf joint_torque = jacobian.transpose() * foot_force;

            return setJointTorques(leg_id, joint_torque);
        }

        /// @brief Get the foot position estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_position Foot position.
        /// @return If the position was retrieved successfully.
        bool getFootPositionEstimate(const uint8_t leg_id,
                                     VectorXf &foot_position)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return false;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            if (!configs_[leg_id].dynamics->forwardKinematics(current_joint_angles, foot_position))
            {
                std::cerr << "Failed to get forward kinematics." << std::endl;
                return false;
            }

            return true;
        }

        /// @brief Get the foot velocity estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_velocity Foot velocity.
        /// @return If the velocity was retrieved successfully.
        bool getFootVelocityEstimate(const uint8_t leg_id,
                                     VectorXf &foot_velocity)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return false;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }

            VectorXf joint_velocities = getCurrentJointVelocities(leg_id);
            foot_velocity = jacobian * joint_velocities;

            return true;
        }

        /// @brief Get the foot force estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_force Foot force.
        /// @return If the force was retrieved successfully.
        bool getFootForceEstimate(const uint8_t leg_id,
                                  VectorXf &foot_force)
        {

            if (leg_id > MAX_LEG_ID)
            {
                std::cerr << "Invalid leg ID." << std::endl;
                return false;
            }

            if (!configs_[leg_id].dynamics)
            {
                std::cerr << "Leg dynamics not set." << std::endl;
                return false;
            }

            VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            MatrixXf jacobian;
            if (!configs_[leg_id].dynamics->jacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }

            VectorXf joint_torques = getCurrentJointTorques(leg_id);
            foot_force = jacobian * joint_torques;

            return true;
        }

    private:
        VectorXf getCurrentJointAngles(const uint8_t leg_id)
        {
            VectorXf joint_angles(configs_[leg_id].motor_can_ids.size());
            for (size_t i = 0; i < configs_[leg_id].motor_can_ids.size(); i++)
            {
                joint_angles(i) = odrive_controller_->getPositionEstimate(configs_[leg_id].motor_can_ids[i]);
            }
            return joint_angles;
        }

        VectorXf getCurrentJointVelocities(const uint8_t leg_id)
        {
            VectorXf joint_velocities(configs_[leg_id].motor_can_ids.size());
            for (size_t i = 0; i < configs_[leg_id].motor_can_ids.size(); i++)
            {
                joint_velocities(i) = odrive_controller_->getVelocityEstimate(configs_[leg_id].motor_can_ids[i]);
            }
            return joint_velocities;
        }

        VectorXf getCurrentJointTorques(const uint8_t leg_id)
        {
            VectorXf joint_torques(configs_[leg_id].motor_can_ids.size());
            for (size_t i = 0; i < configs_[leg_id].motor_can_ids.size(); i++)
            {
                joint_torques(i) = odrive_controller_->getTorqueEstimate(configs_[leg_id].motor_can_ids[i]);
            }
            return joint_torques;
        }

        bool setJointAngles(const uint8_t leg_id,
                            const VectorXf &joint_angles,
                            const VectorXf &joint_velocity_ff = VectorXf(),
                            const VectorXf &joint_torque_ff = VectorXf())
        {
            size_t expected_size = configs_[leg_id].motor_can_ids.size();
            if (joint_angles.size() != expected_size ||
                ((joint_velocity_ff.size() > 0) && (joint_velocity_ff.size() != expected_size)) ||
                ((joint_torque_ff.size() > 0) && (joint_torque_ff.size() != expected_size)))
            {
                std::cerr << "Invalid joint parameters." << std::endl;
                return false;
            }

            bool success = true;
            for (size_t i = 0; i < expected_size; i++)
            {
                float velocity_ff = (joint_velocity_ff.size() > 0) ? joint_velocity_ff(i, 0) : 0.0f;
                float torque_ff = (joint_torque_ff.size() > 0) ? joint_torque_ff(i, 0) : 0.0f;
                success &= odrive_controller_->setPosition(configs_[leg_id].motor_can_ids[i],
                                                           joint_angles(i, 0),
                                                           velocity_ff,
                                                           torque_ff);
            }
            return success;
        }

        bool setJointVelocities(const uint8_t leg_id,
                                const VectorXf &joint_velocities,
                                const VectorXf &joint_torque_ff = VectorXf())
        {
            size_t expected_size = configs_[leg_id].motor_can_ids.size();
            if (joint_velocities.size() != expected_size ||
                ((joint_torque_ff.size() > 0) && (joint_torque_ff.size() != expected_size)))
            {
                std::cerr << "Invalid joint parameters." << std::endl;
                return false;
            }

            bool success = true;
            for (size_t i = 0; i < expected_size; i++)
            {
                float torque_ff = (joint_torque_ff.size() > 0) ? joint_torque_ff(i, 0) : 0.0f;
                success &= odrive_controller_->setVelocity(configs_[leg_id].motor_can_ids[i],
                                                           joint_velocities(i, 0),
                                                           torque_ff);
            }
            return success;
        }

        bool setJointTorques(const uint8_t leg_id,
                             const VectorXf &joint_torques)
        {
            size_t expected_size = configs_[leg_id].motor_can_ids.size();
            if (joint_torques.size() != expected_size)
            {
                std::cerr << "Invalid joint parameters." << std::endl;
                return false;
            }

            bool success = true;
            for (size_t i = 0; i < expected_size; i++)
            {
                success &= odrive_controller_->setTorque(configs_[leg_id].motor_can_ids[i],
                                                         joint_torques(i, 0));
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