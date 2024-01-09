#ifndef STARQ__MOTOR_CONTROLLER_HPP_
#define STARQ__MOTOR_CONTROLLER_HPP_

#include <memory>

#define MAX_MOTOR_ID 0x3F

namespace starq
{

    enum MotorState
    {
        UNDEFINED = 0x0,
        IDLE = 0x1,
        CLOSED_LOOP_CONTROL = 0x8
    };

    enum ControlMode
    {
        VOLTAGE = 0x0,
        TORQUE = 0x1,
        VELOCITY = 0x2,
        POSITION = 0x3
    };

    enum InputMode
    {
        INACTIVE = 0x0,
        PASSTHROUGH = 0x1,
        VEL_RAMP = 0x2,
        POS_FILTER = 0x3,
        TRAP_TRAJ = 0x5,
        TORQUE_RAMP = 0x6
    };

    class MotorController
    {
    public:
        using Ptr = std::shared_ptr<MotorController>;

        /// @brief Set the motor state.
        /// @param motor_id The ID of the motor.
        /// @param state The state to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setState(const uint8_t motor_id, const uint32_t state) = 0;

        /// @brief Set the control mode.
        /// @param motor_id The ID of the motor.
        /// @param control_mode The control mode to set the motor to.
        /// @param input_mode The input mode to set the motor to. (default: 0x1)
        /// @return If the command was sent successfully.
        virtual bool setControlMode(const uint8_t motor_id, const uint32_t control_mode, const uint32_t input_mode = 0x1) = 0;

        /// @brief Set the position.
        /// @param motor_id The ID of the motor.
        /// @param pos The position to set the motor to.
        /// @param vel_ff The velocity feedforward to set the motor to. (default: 0)
        /// @param torque_ff The torque feedforward to set the motor to. (default: 0)
        /// @return If the command was sent successfully.
        virtual bool setPosition(const uint8_t motor_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) = 0;

        /// @brief Set the velocity.
        /// @param motor_id The ID of the motor.
        /// @param vel The velocity to set the motor to.
        /// @param torque_ff The torque feedforward to set the motor to. (default: 0)
        /// @return If the command was sent successfully.
        virtual bool setVelocity(const uint8_t motor_id, const float vel, const float torque_ff = 0.F) = 0;

        /// @brief Set the torque.
        /// @param motor_id The ID of the motor.
        /// @param torque The torque to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setTorque(const uint8_t motor_id, const float torque) = 0;

        /// @brief Get the encoder position estimate.
        /// @param motor_id The ID of the motor.
        /// @return The encoder position estimate.
        virtual float getPositionEstimate(const uint8_t motor_id) = 0;

        /// @brief Get the encoder velocity estimate.
        /// @param motor_id The ID of the motor.
        /// @return The encoder velocity estimate.
        virtual float getVelocityEstimate(const uint8_t motor_id) = 0;

        /// @brief Get the controller torque estimate.
        /// @param motor_id The ID of the motor.
        /// @return The controller torque estimate.
        virtual float getTorqueEstimate(const uint8_t motor_id) = 0;
    };

}

#endif