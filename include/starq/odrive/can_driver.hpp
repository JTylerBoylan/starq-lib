#ifndef STARQ_ODRIVE__CAN_DRIVER_HPP_
#define STARQ_ODRIVE__CAN_DRIVER_HPP_

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

    class ODriveCANDriver
    {
    public:
        using Ptr = std::shared_ptr<ODriveCANDriver>;

        /// @brief Create an ODrive driver.
        /// @param socket CAN socket to use for communication.
        ODriveCANDriver(const starq::can::CANSocket::Ptr socket);

        /// @brief Destroy the ODrive driver.
        ~ODriveCANDriver();

        /// @brief Set the axis state.
        /// @param can_id CAN ID of the axis.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t can_id, const uint32_t state);

        /// @brief Set the control mode.
        /// @param can_id CAN ID of the axis.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode.
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1);

        /// @brief Set the limits.
        /// @param can_id CAN ID of the axis.
        /// @param velocity_limit Velocity limit.
        /// @param current_limit Current limit.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit);

        /// @brief Set the position gain.
        /// @param can_id CAN ID of the axis.
        /// @param pos_gain Position gain.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain);

        /// @brief Set the velocity gains.
        /// @param can_id CAN ID of the axis.
        /// @param vel_gain Velocity gain.
        /// @param vel_integrator_gain Velocity integrator gain.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain);

        /// @brief Set the position.
        /// @param can_id CAN ID of the axis.
        /// @param pos Position.
        /// @param vel_ff Feedforward velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F);

        /// @brief Set the velocity.
        /// @param can_id CAN ID of the axis.
        /// @param vel Velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F);

        /// @brief Set the torque.
        /// @param can_id CAN ID of the axis.
        /// @param torque Torque.
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque);

    private:

        /// @brief Get the arbitration ID.
        /// @param can_id CAN ID of the axis.
        /// @param cmd_id Command ID.
        /// @return Arbitration ID.
        inline uint32_t getArbitrationID(const uint8_t can_id, const uint8_t cmd_id) const
        {
            return (uint32_t)(can_id << 5) | cmd_id;
        }

        const starq::can::CANSocket::Ptr socket_;
    };

}

#endif