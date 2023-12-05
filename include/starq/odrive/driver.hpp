#ifndef STARQ_ODRIVE__DRIVER_HPP_
#define STARQ_ODRIVE__DRIVER_HPP_

#include "starq/can/socket.hpp"

namespace starq::odrive
{

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

    class ODriveDriver
    {
    public:
        using Ptr = std::shared_ptr<ODriveDriver>;

        /// @brief Create an ODrive driver.
        /// @param socket CAN socket to use for communication.
        ODriveDriver(const starq::can::CANSocket::Ptr socket)
            : socket_(socket)
        {
        }

        /// @brief Destroy the ODrive driver.
        ~ODriveDriver()
        {
        }

        /// @brief Set the axis state.
        /// @param can_id CAN ID of the axis.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t can_id, const uint32_t state)
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

        /// @brief Set the control mode.
        /// @param can_id CAN ID of the axis.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode.
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1)
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

        /// @brief Set the limits.
        /// @param can_id CAN ID of the axis.
        /// @param velocity_limit Velocity limit.
        /// @param current_limit Current limit.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
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

        /// @brief Set the position gain.
        /// @param can_id CAN ID of the axis.
        /// @param pos_gain Position gain.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain)
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

        /// @brief Set the velocity gains.
        /// @param can_id CAN ID of the axis.
        /// @param vel_gain Velocity gain.
        /// @param vel_integrator_gain Velocity integrator gain.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain)
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

        /// @brief Set the position.
        /// @param can_id CAN ID of the axis.
        /// @param pos Position.
        /// @param vel_ff Feedforward velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F)
        {
            const int cmd_id = 0x01c;
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

        /// @brief Set the velocity.
        /// @param can_id CAN ID of the axis.
        /// @param vel Velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F)
        {
            const int cmd_id = 0x01d;
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

        /// @brief Set the torque.
        /// @param can_id CAN ID of the axis.
        /// @param torque Torque.
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque)
        {
            const int cmd_id = 0x01e;
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

    private:
        uint32_t getArbitrationID(const uint8_t can_id, const uint8_t cmd_id) const
        {
            return (uint32_t)(can_id << 5) | cmd_id;
        }

        const starq::can::CANSocket::Ptr socket_;
    };

}

#endif