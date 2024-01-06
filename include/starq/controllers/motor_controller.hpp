#ifndef STARQ_CONTROLLERS__MOTOR_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__MOTOR_CONTROLLER_HPP_

#include <memory>
#include <iostream>

#define MAX_MOTOR_ID 0x3F

namespace starq::controllers
{

    enum AxisState
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

        /// @brief Set the axis state.
        /// @param motor_id The ID of the motor.
        /// @param state The state to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setAxisState(const uint8_t motor_id, const uint32_t state) = 0;

        /// @brief Set the control mode.
        /// @param motor_id The ID of the motor.
        /// @param control_mode The control mode to set the motor to.
        /// @param input_mode The input mode to set the motor to. (default: 0x1)
        /// @return If the command was sent successfully.
        virtual bool setControlMode(const uint8_t motor_id, const uint32_t control_mode, const uint32_t input_mode = 0x1) = 0;

        /// @brief Set the position gain.
        /// @param motor_id The ID of the motor.
        /// @param pos_gain The position gain to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setPosGain(const uint8_t motor_id, const float pos_gain) = 0;

        /// @brief Set the velocity gain.
        /// @param motor_id The ID of the motor.
        /// @param vel_gain The velocity gain to set the motor to.
        /// @param vel_integrator_gain The velocity integrator gain to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setVelGains(const uint8_t motor_id, const float vel_gain, const float vel_integrator_gain) = 0;

        /// @brief Set the velocity limit.
        /// @param motor_id The ID of the motor.
        /// @param velocity_limit The velocity limit to set the motor to.
        /// @param current_limit The current limit to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setLimits(const uint8_t motor_id, const float velocity_limit, const float current_limit) = 0;

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

        /// @brief Get the axis error.
        /// @param motor_id The ID of the motor.
        /// @return The axis error.
        virtual uint32_t getAxisError(const uint8_t motor_id) = 0;

        /// @brief Get the axis state.
        /// @param motor_id The ID of the motor.
        /// @return The axis state.
        virtual uint8_t getAxisState(const uint8_t motor_id) = 0;

        /// @brief Get the Iq setpoint.
        /// @param motor_id The ID of the motor.
        /// @return The Iq setpoint.
        virtual float getIqSetpoint(const uint8_t motor_id) = 0;

        /// @brief Get the Iq measured.
        /// @param motor_id The ID of the motor.
        /// @return The Iq measured.
        virtual float getIqMeasured(const uint8_t motor_id) = 0;

        /// @brief Get the FET temperature.
        /// @param motor_id The ID of the motor.
        /// @return The FET temperature.
        virtual float getFETTemperature(const uint8_t motor_id) = 0;

        /// @brief Get the motor temperature.
        /// @param motor_id The ID of the motor.
        /// @return The motor temperature.
        virtual float getMotorTemperature(const uint8_t motor_id) = 0;

        /// @brief Get the DC bus voltage.
        /// @param motor_id The ID of the motor.
        /// @return The DC bus voltage.
        virtual float getBusVoltage(const uint8_t motor_id) = 0;

        /// @brief Get the DC bus current.
        /// @param motor_id The ID of the motor.
        /// @return The DC bus current.
        virtual float getBusCurrent(const uint8_t motor_id) = 0;

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

        /// @brief Get the axis state config.
        /// @param motor_id The ID of the motor.
        /// @return The axis state config.
        uint32_t getAxisStateConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].axis_state;
        }

        /// @brief Get the control mode config.
        /// @param motor_id The ID of the motor.
        /// @return The control mode config.
        uint32_t getControlModeConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].control_mode;
        }

        /// @brief Get the input mode config.
        /// @param motor_id The ID of the motor.
        /// @return The input mode config.
        uint32_t getInputModeConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].input_mode;
        }

        /// @brief Get the position gain config.
        /// @param motor_id The ID of the motor.
        /// @return The position gain config.
        float getPosGainConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].pos_gain;
        }

        /// @brief Get the velocity gain config.
        /// @param motor_id The ID of the motor.
        /// @return The velocity gain config.
        float getVelGainConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].vel_gain;
        }

        /// @brief Get the velocity integrator gain config.
        /// @param motor_id The ID of the motor.
        /// @return The velocity integrator gain config.
        float getIntegratorGainConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].integrator_gain;
        }

        /// @brief Get the velocity limit config.
        /// @param motor_id The ID of the motor.
        /// @return The velocity limit config.
        float getVelocityLimitConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].velocity_limit;
        }

        /// @brief Get the current limit config.
        /// @param motor_id The ID of the motor.
        /// @return The current limit config.
        float getCurrentLimitConfig(const uint8_t motor_id) const
        {
            return configs_[motor_id].current_limit;
        }

        /// @brief Print the motor info.
        /// @param motor_id The ID of the motor.
        void printInfo(const uint8_t motor_id)
        {
            std::cout << "Motor " << (int)motor_id << " info:" << std::endl;
            std::cout << "  Axis error: " << getAxisError(motor_id) << std::endl;
            std::cout << "  Axis state: " << (int)getAxisState(motor_id) << std::endl;
            std::cout << "  Iq setpoint: " << getIqSetpoint(motor_id) << std::endl;
            std::cout << "  Iq measured: " << getIqMeasured(motor_id) << std::endl;
            std::cout << "  FET temperature: " << getFETTemperature(motor_id) << std::endl;
            std::cout << "  Motor temperature: " << getMotorTemperature(motor_id) << std::endl;
            std::cout << "  Bus voltage: " << getBusVoltage(motor_id) << std::endl;
            std::cout << "  Bus current: " << getBusCurrent(motor_id) << std::endl;
            std::cout << "  Position estimate: " << getPositionEstimate(motor_id) << std::endl;
            std::cout << "  Velocity estimate: " << getVelocityEstimate(motor_id) << std::endl;
            std::cout << "  Torque estimate: " << getTorqueEstimate(motor_id) << std::endl;
        }

        /// @brief Print the motor config.
        /// @param motor_id The ID of the motor.
        void printConfig(const uint8_t motor_id)
        {
            std::cout << "Motor " << (int)motor_id << " config:" << std::endl;
            std::cout << "  Axis state: " << (int)getAxisStateConfig(motor_id) << std::endl;
            std::cout << "  Control mode: " << (int)getControlModeConfig(motor_id) << std::endl;
            std::cout << "  Input mode: " << (int)getInputModeConfig(motor_id) << std::endl;
            std::cout << "  Position gain: " << getPosGainConfig(motor_id) << std::endl;
            std::cout << "  Velocity gain: " << getVelGainConfig(motor_id) << std::endl;
            std::cout << "  Integrator gain: " << getIntegratorGainConfig(motor_id) << std::endl;
            std::cout << "  Velocity limit: " << getVelocityLimitConfig(motor_id) << std::endl;
            std::cout << "  Current limit: " << getCurrentLimitConfig(motor_id) << std::endl;
        }

    protected:
        struct
        {
            uint32_t axis_state = 0;
            uint32_t control_mode = 0;
            uint32_t input_mode = 0;
            float pos_gain = 0.0f;
            float vel_gain = 0.0f;
            float integrator_gain = 0.0f;
            float velocity_limit = 0.0f;
            float current_limit = 0.0f;
        } configs_[MAX_MOTOR_ID + 1];
    };

}

#endif