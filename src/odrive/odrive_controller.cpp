#include "starq/odrive/odrive_controller.hpp"

#include <iostream>
#include <cmath>

namespace starq::odrive
{

    ODriveController::ODriveController(const starq::can::CANSocket::Ptr socket)
        : driver_(std::make_shared<ODriveCANDriver>(socket)),
          listener_(std::make_shared<ODriveCANListener>(socket))
    {
    }

    ODriveController::~ODriveController()
    {
    }

    bool ODriveController::setGearRatio(const uint8_t motor_id, const float gear_ratio)
    {
        if (motor_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid motor ID." << std::endl;
            return false;
        }

        configs_[motor_id].gear_ratio = gear_ratio;
        return true;
    }

    bool ODriveController::setState(const uint8_t can_id, const uint32_t state)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        if (configs_[can_id].axis_state == state)
            return true;

        if (!driver_->setAxisState(can_id, state))
        {
            std::cerr << "Failed to set axis state." << std::endl;
            return false;
        }

        configs_[can_id].axis_state = state;
        return true;
    }

    bool ODriveController::setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        if (configs_[can_id].control_mode == control_mode && configs_[can_id].input_mode == input_mode)
            return true;

        if (!driver_->setControlMode(can_id, control_mode, input_mode))
        {
            std::cerr << "Failed to set control/input mode." << std::endl;
            return false;
        }

        configs_[can_id].control_mode = control_mode;
        configs_[can_id].input_mode = input_mode;
        return true;
    }

    bool ODriveController::setPosGain(const uint8_t can_id, const float pos_gain)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        if (configs_[can_id].pos_gain == pos_gain)
            return true;

        if (!driver_->setPosGain(can_id, pos_gain))
        {
            std::cerr << "Failed to set position gain." << std::endl;
            return false;
        }

        configs_[can_id].pos_gain = pos_gain;
        return true;
    }

    bool ODriveController::setVelGains(const uint8_t can_id, const float vel_gain, const float integrator_gain)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        if (configs_[can_id].vel_gain == vel_gain && configs_[can_id].integrator_gain == integrator_gain)
            return true;

        if (!driver_->setVelGains(can_id, vel_gain, integrator_gain))
        {
            std::cerr << "Failed to set velocity gains." << std::endl;
            return false;
        }

        configs_[can_id].vel_gain = vel_gain;
        configs_[can_id].integrator_gain = integrator_gain;
        return true;
    }

    bool ODriveController::setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        if (configs_[can_id].velocity_limit == velocity_limit && configs_[can_id].current_limit == current_limit)
            return true;

        if (!driver_->setLimits(can_id, velocity_limit, current_limit))
        {
            std::cerr << "Failed to set limits." << std::endl;
            return false;
        }

        configs_[can_id].velocity_limit = velocity_limit;
        configs_[can_id].current_limit = current_limit;
        return true;
    }

    bool ODriveController::setPosition(const uint8_t can_id, const float pos, const float vel_ff, const float torque_ff)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Convert from radians to revolutions and apply gear ratio
        const float pos_rev = configs_[can_id].gear_ratio * pos / (2.0f * M_PI);
        const float vel_ff_rev = configs_[can_id].gear_ratio * vel_ff / (2.0f * M_PI);
        const float torque_ff_N = torque_ff / configs_[can_id].gear_ratio;

        return driver_->setPosition(can_id, pos_rev, vel_ff_rev, torque_ff_N);
    }

    bool ODriveController::setVelocity(const uint8_t can_id, const float vel, const float torque_ff)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Convert from radians to revolutions and apply gear ratio
        const float vel_rev = configs_[can_id].gear_ratio * vel / (2.0f * M_PI);
        const float torque_ff_N = torque_ff / configs_[can_id].gear_ratio;

        return driver_->setVelocity(can_id, vel_rev, torque_ff_N);
    }

    bool ODriveController::setTorque(const uint8_t can_id, const float torque)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Apply gear ratio
        const float torque_N = torque / configs_[can_id].gear_ratio;

        return driver_->setTorque(can_id, torque_N);
    }

    uint32_t ODriveController::getAxisError(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getAxisError(can_id);
    }

    uint8_t ODriveController::getAxisState(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getAxisState(can_id);
    }

    float ODriveController::getIqSetpoint(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getIqSetpoint(can_id);
    }

    float ODriveController::getIqMeasured(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getIqMeasured(can_id);
    }

    float ODriveController::getFETTemperature(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getFETTemperature(can_id);
    }

    float ODriveController::getMotorTemperature(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getMotorTemperature(can_id);
    }

    float ODriveController::getBusVoltage(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getBusVoltage(can_id);
    }

    float ODriveController::getBusCurrent(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        return listener_->getBusCurrent(can_id);
    }

    float ODriveController::getPositionEstimate(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Convert from revolutions to radians and apply gear ratio
        return listener_->getPosEstimate(can_id) * (2.0f * M_PI) / configs_[can_id].gear_ratio;
    }

    float ODriveController::getVelocityEstimate(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Convert from revolutions to radians and apply gear ratio
        return listener_->getVelEstimate(can_id) * (2.0f * M_PI) / configs_[can_id].gear_ratio;
    }

    float ODriveController::getTorqueEstimate(const uint8_t can_id)
    {
        if (can_id > MAX_MOTOR_ID)
        {
            std::cerr << "Invalid CAN ID." << std::endl;
            return false;
        }

        // Apply gear ratio
        return listener_->getTorqueEstimate(can_id) * configs_[can_id].gear_ratio;
    }

    void ODriveController::printInfo(const uint8_t motor_id)
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

}