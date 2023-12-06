#ifndef STARQ_CONTROLLERS__LEG_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__LEG_CONTROLLER_HPP_

#include <vector>

#include "starq/controllers/motor_controller.hpp"
#include "starq/dynamics/leg_dynamics.hpp"

#define MAX_LEG_ID 0x3F

namespace starq::controllers
{
    using namespace Eigen;

    class LegController
    {

    public:
        using Ptr = std::shared_ptr<LegController>;

        /// @brief Create a leg controller.
        /// @param controller Motor controller.
        LegController(const starq::controllers::MotorController::Ptr motor_controller);

        /// @brief Destroy the leg controller.
        ~LegController();

        /// @brief Set the motor IDs for a leg.
        /// @param leg_id ID of the leg.
        /// @param motor_ids IDs of the motors.
        void setMotorIDs(const uint8_t leg_id, const std::vector<uint32_t> motor_ids);

        /// @brief Set the leg dynamics for a leg.
        /// @param leg_id ID of the leg.
        /// @param dynamics Leg dynamics.
        void setLegDynamics(const uint8_t leg_id, const starq::dynamics::LegDynamics::Ptr dynamics);

        /// @brief Set the axis state for a leg.
        /// @param leg_id ID of the leg.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t leg_id, const uint32_t state);

        /// @brief Set the control mode for a leg.
        /// @param leg_id ID of the leg.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t leg_id, const uint32_t control_mode, const uint32_t input_mode = 0x1);

        /// @brief Set the position gain for a leg.
        /// @param leg_id ID of the leg.
        /// @param pos_gain Position gain.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t leg_id, const float pos_gain);

        /// @brief Set the velocity gain for a leg.
        /// @param leg_id ID of the leg.
        /// @param vel_gain Velocity gain.
        /// @param vel_integrator_gain Velocity integrator gain.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t leg_id, const float vel_gain, const float vel_integrator_gain);

        /// @brief Set the limits for a leg.
        /// @param leg_id ID of the leg.
        /// @param vel_limit Velocity limit.
        /// @param current_limit Current limit.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t leg_id, const float vel_limit, const float current_limit);

        /// @brief Set the foot position for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_position Foot position.
        /// @param foot_velocity_ff Foot velocity feedforward.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return If the command was sent successfully.
        bool setFootPosition(const uint8_t leg_id,
                             const VectorXf &foot_position,
                             const VectorXf &foot_velocity_ff = VectorXf(),
                             const VectorXf &foot_torque_ff = VectorXf());

        /// @brief Set the foot velocity for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_velocity Foot velocity.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return  If the command was sent successfully.
        bool setFootVelocity(const uint8_t leg_id,
                             const VectorXf &foot_velocity,
                             const VectorXf &foot_torque_ff = VectorXf());

        /// @brief Set the foot force for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_force Foot force.
        /// @return If the command was sent successfully.
        bool setFootForce(const uint8_t leg_id,
                          const VectorXf &foot_force);

        /// @brief Get the foot position estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_position Foot position.
        /// @return If the position was retrieved successfully.
        bool getFootPositionEstimate(const uint8_t leg_id,
                                     VectorXf &foot_position);

        /// @brief Get the foot velocity estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_velocity Foot velocity.
        /// @return If the velocity was retrieved successfully.
        bool getFootVelocityEstimate(const uint8_t leg_id,
                                     VectorXf &foot_velocity);

        /// @brief Get the foot force estimate for a leg.
        /// @param leg_id ID of the leg.
        /// @param foot_force Foot force.
        /// @return If the force was retrieved successfully.
        bool getFootForceEstimate(const uint8_t leg_id,
                                  VectorXf &foot_force);

    private:

        /// @brief Get the current joint angles for a leg.
        /// @param leg_id Leg ID.
        /// @return Vector of joint angles.
        VectorXf getCurrentJointAngles(const uint8_t leg_id);

        /// @brief Get the current joint velocities for a leg.
        /// @param leg_id Leg ID.
        /// @return Vector of joint velocities.
        VectorXf getCurrentJointVelocities(const uint8_t leg_id);

        /// @brief Get the current joint torques for a leg.
        /// @param leg_id Leg ID.
        /// @return Vector of joint torques.
        VectorXf getCurrentJointTorques(const uint8_t leg_id);

        /// @brief Set the joint angles for a leg.
        /// @param leg_id Leg ID.
        /// @param joint_angles Joint angles.
        /// @param joint_velocity_ff Joint velocity feedforward. (default: empty)
        /// @param joint_torque_ff Joint torque feedforward (default: empty)
        /// @return If the command was sent successfully.
        bool setJointAngles(const uint8_t leg_id,
                            const VectorXf &joint_angles,
                            const VectorXf &joint_velocity_ff = VectorXf(),
                            const VectorXf &joint_torque_ff = VectorXf());

        /// @brief Set the joint velocities for a leg.
        /// @param leg_id Leg ID.
        /// @param joint_velocities Joint velocities.
        /// @param joint_torque_ff Joint torque feedforward (default: empty)
        /// @return If the command was sent successfully.
        bool setJointVelocities(const uint8_t leg_id,
                                const VectorXf &joint_velocities,
                                const VectorXf &joint_torque_ff = VectorXf());

        /// @brief Set the joint torques for a leg.
        /// @param leg_id Leg ID.
        /// @param joint_torques Joint torques.
        /// @return If the command was sent successfully.
        bool setJointTorques(const uint8_t leg_id,
                             const VectorXf &joint_torques);

        const starq::controllers::MotorController::Ptr motor_controller_;

        struct
        {
            std::vector<uint32_t> motor_ids;
            starq::dynamics::LegDynamics::Ptr dynamics;
        } configs_[MAX_LEG_ID + 1];
    };

}

#endif