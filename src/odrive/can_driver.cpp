#include "starq/odrive/can_driver.hpp"

namespace starq::odrive
{

    ODriveCANDriver::ODriveCANDriver(const starq::can::CANSocket::Ptr socket)
        : socket_(socket)
    {
    }

    ODriveCANDriver::~ODriveCANDriver()
    {
    }

    bool ODriveCANDriver::setAxisState(const uint8_t can_id, const uint32_t state)
    {
        const int cmd_id = 0x007;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &state, sizeof(state));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send axis state." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode)
    {
        const int cmd_id = 0x00b;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &control_mode, sizeof(control_mode));
        std::memcpy(data + 4, &input_mode, sizeof(input_mode));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send control mode." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
    {
        const int cmd_id = 0x00f;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &velocity_limit, sizeof(velocity_limit));
        std::memcpy(data + 4, &current_limit, sizeof(current_limit));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send limits." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setPosGain(const uint8_t can_id, const float pos_gain)
    {
        const int cmd_id = 0x01a;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &pos_gain, sizeof(pos_gain));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send position gain." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain)
    {
        const int cmd_id = 0x01b;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &vel_gain, sizeof(vel_gain));
        std::memcpy(data + 4, &vel_integrator_gain, sizeof(vel_integrator_gain));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send velocity gains." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setPosition(const uint8_t can_id, const float pos, const float vel_ff, const float torque_ff)
    {
        const int cmd_id = 0x00c;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        const int16_t vel_ff_int = (int16_t)(vel_ff * 1E3F);
        const int16_t torque_ff_int = (int16_t)(torque_ff * 1E3F);

        uint8_t data[8];
        std::memcpy(data, &pos, sizeof(pos));
        std::memcpy(data + 4, &vel_ff, sizeof(vel_ff_int));
        std::memcpy(data + 6, &torque_ff, sizeof(torque_ff_int));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send position." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setVelocity(const uint8_t can_id, const float vel, const float torque_ff)
    {
        const int cmd_id = 0x00d;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &vel, sizeof(vel));
        std::memcpy(data + 4, &torque_ff, sizeof(torque_ff));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send velocity." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveCANDriver::setTorque(const uint8_t can_id, const float torque)
    {
        const int cmd_id = 0x00e;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &torque, sizeof(torque));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send torque." << std::endl;
            return false;
        }
        return true;
    }

}