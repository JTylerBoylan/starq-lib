#include "starq/odrive/odrive_controller.hpp"

namespace starq::odrive
{

    ODriveController::ODriveController(const starq::can::CANSocket::Ptr socket)
        : driver_(std::make_shared<ODriveDriver>(socket)),
          listener_(std::make_shared<ODriveListener>(socket))
    {
    }

    ODriveController::~ODriveController()
    {
    }

    bool ODriveController::setAxisState(const uint8_t can_id, const uint32_t state)
    {
        if (driver_->setAxisState(can_id, state))
        {
            configs_[can_id].axis_state = state;
            return true;
        }
        return false;
    }

    bool ODriveController::setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode)
    {
        if (driver_->setControlMode(can_id, control_mode, input_mode))
        {
            configs_[can_id].control_mode = control_mode;
            configs_[can_id].input_mode = input_mode;
            return true;
        }
        return false;
    }

    bool ODriveController::setPosGain(const uint8_t can_id, const float pos_gain)
    {
        if (driver_->setPosGain(can_id, pos_gain))
        {
            configs_[can_id].pos_gain = pos_gain;
            return true;
        }
        return false;
    }

    bool ODriveController::setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain)
    {
        if (driver_->setVelGains(can_id, vel_gain, vel_integrator_gain))
        {
            configs_[can_id].vel_gain = vel_gain;
            configs_[can_id].vel_integrator_gain = vel_integrator_gain;
            return true;
        }
        return false;
    }

    bool ODriveController::setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
    {
        if (driver_->setLimits(can_id, velocity_limit, current_limit))
        {
            configs_[can_id].velocity_limit = velocity_limit;
            configs_[can_id].current_limit = current_limit;
            return true;
        }
        return false;
    }

    bool ODriveController::setPosition(const uint8_t can_id, const float pos, const float vel_ff, const float torque_ff)
    {
        if (configs_[can_id].control_mode != 0x3)
        {
            std::cerr << "Control mode is not position control." << std::endl;
            return false;
        }
        return driver_->setPosition(can_id, pos, vel_ff, torque_ff);
    }

    bool ODriveController::setVelocity(const uint8_t can_id, const float vel, const float torque_ff)
    {
        if (configs_[can_id].control_mode != 0x2)
        {
            std::cerr << "Control mode is not velocity control." << std::endl;
            return false;
        }
        return driver_->setVelocity(can_id, vel, torque_ff);
    }

    bool ODriveController::setTorque(const uint8_t can_id, const float torque)
    {
        if (configs_[can_id].control_mode != 0x1)
        {
            std::cerr << "Control mode is not torque control." << std::endl;
            return false;
        }
        return driver_->setTorque(can_id, torque);
    }

    uint32_t ODriveController::getAxisError(const uint8_t can_id)
    {
        return listener_->getAxisError(can_id);
    }

    uint8_t ODriveController::getAxisState(const uint8_t can_id)
    {
        return listener_->getAxisState(can_id);
    }

    float ODriveController::getIqSetpoint(const uint8_t can_id)
    {
        return listener_->getIqSetpoint(can_id);
    }

    float ODriveController::getIqMeasured(const uint8_t can_id)
    {
        return listener_->getIqMeasured(can_id);
    }

    float ODriveController::getFETTemperature(const uint8_t can_id)
    {
        return listener_->getFETTemperature(can_id);
    }

    float ODriveController::getMotorTemperature(const uint8_t can_id)
    {
        return listener_->getMotorTemperature(can_id);
    }

    float ODriveController::getBusVoltage(const uint8_t can_id)
    {
        return listener_->getBusVoltage(can_id);
    }

    float ODriveController::getBusCurrent(const uint8_t can_id)
    {
        return listener_->getBusCurrent(can_id);
    }

    float ODriveController::getPositionEstimate(const uint8_t can_id)
    {
        return listener_->getPosEstimate(can_id);
    }

    float ODriveController::getVelocityEstimate(const uint8_t can_id)
    {
        return listener_->getVelEstimate(can_id);
    }

    float ODriveController::getTorqueEstimate(const uint8_t can_id)
    {
        return listener_->getTorqueEstimate(can_id);
    }

}