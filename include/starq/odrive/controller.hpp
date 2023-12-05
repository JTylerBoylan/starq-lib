#ifndef STARQ_ODRIVE__CONTROLLER_HPP_
#define STARQ_ODRIVE__CONTROLLER_HPP_

#include "starq/odrive/driver.hpp"
#include "starq/odrive/listener.hpp"

namespace starq::odrive
{

    class ODriveController
    {
    public:
        using Ptr = std::shared_ptr<ODriveController>;

        /// @brief Connect to ODrive via CAN
        /// @param socket The CAN socket.
        ODriveController(const starq::can::CANSocket::Ptr socket)
            : driver_(std::make_shared<ODriveDriver>(socket)),
              listener_(std::make_shared<ODriveListener>(socket))
        {
        }

        /// @brief Disconnect from ODrive.
        ~ODriveController()
        {
        }

        /// @brief Set the axis state.
        /// @param can_id The CAN ID of the ODrive.
        /// @param state The state to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t can_id, const uint32_t state)
        {
            if (driver_->setAxisState(can_id, state))
            {
                configs_[can_id].axis_state = state;
                return true;
            }
            return false;
        }

        /// @brief Set the control mode.
        /// @param can_id The CAN ID of the ODrive.
        /// @param control_mode The control mode to set the ODrive to.
        /// @param input_mode The input mode to set the ODrive to. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1)
        {
            if (driver_->setControlMode(can_id, control_mode, input_mode))
            {
                configs_[can_id].control_mode = control_mode;
                configs_[can_id].input_mode = input_mode;
                return true;
            }
            return false;
        }

        /// @brief Set the position gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos_gain The position gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain)
        {
            if (driver_->setPosGain(can_id, pos_gain))
            {
                configs_[can_id].pos_gain = pos_gain;
                return true;
            }
            return false;
        }

        /// @brief Set the velocity gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel_gain The velocity gain to set the ODrive to.
        /// @param vel_integrator_gain The velocity integrator gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain)
        {
            if (driver_->setVelGains(can_id, vel_gain, vel_integrator_gain))
            {
                configs_[can_id].vel_gain = vel_gain;
                configs_[can_id].vel_integrator_gain = vel_integrator_gain;
                return true;
            }
            return false;
        }

        /// @brief Set the velocity limit.
        /// @param can_id The CAN ID of the ODrive.
        /// @param velocity_limit The velocity limit to set the ODrive to.
        /// @param current_limit The current limit to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
        {
            if (driver_->setLimits(can_id, velocity_limit, current_limit))
            {
                configs_[can_id].velocity_limit = velocity_limit;
                configs_[can_id].current_limit = current_limit;
                return true;
            }
            return false;
        }

        /// @brief Set the position.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos The position to set the ODrive to.
        /// @param vel_ff The velocity feedforward to set the ODrive to. (default: 0)
        /// @param torque_ff The torque feedforward to set the ODrive to. (default: 0)
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F)
        {
            if (configs_[can_id].control_mode != 0x3)
            {
                std::cerr << "Control mode is not position control." << std::endl;
                return false;
            }
            return driver_->setPosition(can_id, pos, vel_ff, torque_ff);
        }

        /// @brief Set the velocity.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel The velocity to set the ODrive to.
        /// @param torque_ff The torque feedforward to set the ODrive to. (default: 0)
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F)
        {
            if (configs_[can_id].control_mode != 0x2)
            {
                std::cerr << "Control mode is not velocity control." << std::endl;
                return false;
            }
            return driver_->setVelocity(can_id, vel, torque_ff);
        }

        /// @brief Set the torque.
        /// @param can_id The CAN ID of the ODrive.
        /// @param torque The torque to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque)
        {
            if (configs_[can_id].control_mode != 0x1)
            {
                std::cerr << "Control mode is not torque control." << std::endl;
                return false;
            }
            return driver_->setTorque(can_id, torque);
        }

        /// @brief Get the axis error.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The axis error.
        uint32_t getAxisError(const uint8_t can_id)
        {
            return listener_->getAxisError(can_id);
        }

        /// @brief Get the axis state.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The axis state.
        uint8_t getAxisState(const uint8_t can_id)
        {
            return listener_->getAxisState(can_id);
        }

        /// @brief Get the Iq setpoint.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The Iq setpoint.
        float getIqSetpoint(const uint8_t can_id)
        {
            return listener_->getIqSetpoint(can_id);
        }

        /// @brief Get the Iq measured.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The Iq measured.
        float getIqMeasured(const uint8_t can_id)
        {
            return listener_->getIqMeasured(can_id);
        }

        /// @brief Get the FET temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The FET temperature.
        float getFETTemperature(const uint8_t can_id)
        {
            return listener_->getFETTemperature(can_id);
        }

        /// @brief Get the motor temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The motor temperature.
        float getMotorTemperature(const uint8_t can_id)
        {
            return listener_->getMotorTemperature(can_id);
        }

        /// @brief Get the DC bus voltage.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus voltage.
        float getBusVoltage(const uint8_t can_id)
        {
            return listener_->getBusVoltage(can_id);
        }

        /// @brief Get the DC bus current.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus current.
        float getBusCurrent(const uint8_t can_id)
        {
            return listener_->getBusCurrent(can_id);
        }

        /// @brief Get the encoder position estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder position estimate.
        float getPositionEstimate(const uint8_t can_id)
        {
            return listener_->getPosEstimate(can_id);
        }

        /// @brief Get the encoder velocity estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder velocity estimate.
        float getVelocityEstimate(const uint8_t can_id)
        {
            return listener_->getVelEstimate(can_id);
        }

        /// @brief Get the controller torque estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The controller torque estimate.
        float getTorqueEstimate(const uint8_t can_id)
        {
            return listener_->getTorqueEstimate(can_id);
        }

        /// @brief Get the axis state config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The axis state config.
        uint32_t getAxisStateConfig(const uint8_t can_id) const
        {
            return configs_[can_id].axis_state;
        }

        /// @brief Get the control mode config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The control mode config.
        uint32_t getControlModeConfig(const uint8_t can_id) const
        {
            return configs_[can_id].control_mode;
        }

        /// @brief Get the input mode config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The input mode config.
        uint32_t getInputModeConfig(const uint8_t can_id) const
        {
            return configs_[can_id].input_mode;
        }

        /// @brief Get the position gain config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The position gain config.
        float getPosGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].pos_gain;
        }

        /// @brief Get the velocity gain config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The velocity gain config.
        float getVelGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].vel_gain;
        }

        /// @brief Get the velocity integrator gain config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The velocity integrator gain config.
        float getVelIntegratorGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].vel_integrator_gain;
        }

        /// @brief Get the velocity limit config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The velocity limit config.
        float getVelocityLimitConfig(const uint8_t can_id) const
        {
            return configs_[can_id].velocity_limit;
        }

        /// @brief Get the current limit config.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The current limit config.
        float getCurrentLimitConfig(const uint8_t can_id) const
        {
            return configs_[can_id].current_limit;
        }

    private:
        ODriveDriver::Ptr driver_;
        ODriveListener::Ptr listener_;

        struct
        {
            uint32_t axis_state = 0;
            uint32_t control_mode = 0;
            uint32_t input_mode = 0;
            float pos_gain = 0.0f;
            float vel_gain = 0.0f;
            float vel_integrator_gain = 0.0f;
            float velocity_limit = 0.0f;
            float current_limit = 0.0f;
        } configs_[MAX_CAN_ID + 1];
    };

}

#endif