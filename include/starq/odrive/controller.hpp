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

        ODriveController(const starq::can::CANSocket::Ptr socket)
            : driver_(std::make_shared<ODriveDriver>(socket)),
              listener_(std::make_shared<ODriveListener>(socket))
        {
        }

        ~ODriveController()
        {
        }

        bool setAxisState(const uint8_t can_id, const uint32_t state)
        {
            if (driver_->setAxisState(can_id, state))
            {
                configs_[can_id].axis_state = state;
                return true;
            }
            return false;
        }

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

        bool setPosGain(const uint8_t can_id, const float pos_gain)
        {
            if (driver_->setPosGain(can_id, pos_gain))
            {
                configs_[can_id].pos_gain = pos_gain;
                return true;
            }
            return false;
        }

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

        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F)
        {
            if (configs_[can_id].control_mode != 0x3)
            {
                std::cerr << "Control mode is not position control." << std::endl;
                return false;
            }
            return driver_->setPosition(can_id, pos, vel_ff, torque_ff);
        }

        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F)
        {
            if (configs_[can_id].control_mode != 0x2)
            {
                std::cerr << "Control mode is not velocity control." << std::endl;
                return false;
            }
            return driver_->setVelocity(can_id, vel, torque_ff);
        }

        bool setTorque(const uint8_t can_id, const float torque)
        {
            if (configs_[can_id].control_mode != 0x1)
            {
                std::cerr << "Control mode is not torque control." << std::endl;
                return false;
            }
            return driver_->setTorque(can_id, torque);
        }

        uint32_t getAxisError(const uint8_t can_id) const
        {
            return listener_->getAxisError(can_id);
        }

        uint8_t getAxisState(const uint8_t can_id) const
        {
            return listener_->getAxisState(can_id);
        }

        float getIqSetpoint(const uint8_t can_id) const
        {
            return listener_->getIqSetpoint(can_id);
        }

        float getIqMeasured(const uint8_t can_id) const
        {
            return listener_->getIqMeasured(can_id);
        }

        float getFETTemperature(const uint8_t can_id) const
        {
            return listener_->getFETTemperature(can_id);
        }

        float getMotorTemperature(const uint8_t can_id) const
        {
            return listener_->getMotorTemperature(can_id);
        }

        float getBusVoltage(const uint8_t can_id) const
        {
            return listener_->getBusVoltage(can_id);
        }

        float getBusCurrent(const uint8_t can_id) const
        {
            return listener_->getBusCurrent(can_id);
        }

        float getPositionEstimate(const uint8_t can_id) const
        {
            return listener_->getPosEstimate(can_id);
        }

        float getVelocityEstimate(const uint8_t can_id) const
        {
            return listener_->getVelEstimate(can_id);
        }

        float getTorqueEstimate(const uint8_t can_id) const
        {
            return listener_->getTorqueEstimate(can_id);
        }

        uint32_t getAxisStateConfig(const uint8_t can_id) const
        {
            return configs_[can_id].axis_state;
        }

        uint32_t getControlModeConfig(const uint8_t can_id) const
        {
            return configs_[can_id].control_mode;
        }

        uint32_t getInputModeConfig(const uint8_t can_id) const
        {
            return configs_[can_id].input_mode;
        }

        float getPosGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].pos_gain;
        }

        float getVelGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].vel_gain;
        }

        float getVelIntegratorGainConfig(const uint8_t can_id) const
        {
            return configs_[can_id].vel_integrator_gain;
        }

        float getVelocityLimitConfig(const uint8_t can_id) const
        {
            return configs_[can_id].velocity_limit;
        }

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