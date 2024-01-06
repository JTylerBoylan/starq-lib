#ifndef STARQ_ODRIVE__CAN_LISTENER_HPP_
#define STARQ_ODRIVE__CAN_LISTENER_HPP_

#include <mutex>

#include "starq/can/socket.hpp"

namespace starq::odrive
{

    class ODriveCANListener
    {
    public:
        using Ptr = std::shared_ptr<ODriveCANListener>;

        /// @brief Create an ODrive listener.
        /// @param socket CAN Socket to listen on.
        ODriveCANListener(const starq::can::CANSocket::Ptr socket);

        /// @brief Destroy the ODrive listener.
        ~ODriveCANListener();

        /// @brief Get the axis error.
        /// @param can_id CAN ID of the axis.
        /// @return Axis error.
        uint32_t getAxisError(const uint8_t can_id);

        /// @brief Get the axis state.
        /// @param can_id CAN ID of the axis.
        /// @return Axis state.
        uint8_t getAxisState(const uint8_t can_id);

        /// @brief Get the Iq setpoint.
        /// @param can_id CAN ID of the axis.
        /// @return Iq setpoint.
        float getIqSetpoint(const uint8_t can_id);

        /// @brief Get the Iq measured.
        /// @param can_id CAN ID of the axis.
        /// @return Iq measured.
        float getIqMeasured(const uint8_t can_id);

        /// @brief Get the FET temperature.
        /// @param can_id CAN ID of the axis.
        /// @return FET temperature.
        float getFETTemperature(const uint8_t can_id);

        /// @brief Get the motor temperature.
        /// @param can_id CAN ID of the axis.
        /// @return Motor temperature.
        float getMotorTemperature(const uint8_t can_id);

        /// @brief Get the bus voltage.
        /// @param can_id CAN ID of the axis.
        /// @return Bus voltage.
        float getBusVoltage(const uint8_t can_id);

        /// @brief Get the bus current.
        /// @param can_id CAN ID of the axis.
        /// @return Bus current.
        float getBusCurrent(const uint8_t can_id);

        /// @brief Get the position estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Position estimate.
        float getPosEstimate(const uint8_t can_id);

        /// @brief Get the velocity estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Velocity estimate.
        float getVelEstimate(const uint8_t can_id);

        /// @brief Get the torque estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Torque estimate.
        float getTorqueEstimate(const uint8_t can_id);

    private:

        /// @brief Start the listener.
        /// @return If the listener was started successfully.
        bool start();

        /// @brief Stop the listener.
        /// @return If the listener was stopped successfully.
        bool stop();

        /// @brief Run the listener. (Called by the poll thread.)
        void run();

        /// @brief Get the CAN ID.
        /// @param arb_id Arbitration ID.
        /// @return CAN ID.
        uint8_t getCanID(const uint32_t arb_id) const
        {
            return (uint8_t)((arb_id >> 5) & 0b111111);
        }

        /// @brief Get the command ID.
        /// @param arb_id Arbitration ID.
        /// @return Command ID.
        uint8_t getCommandID(const uint32_t arb_id) const
        {
            return (uint8_t)(arb_id & 0b11111);
        }

        const starq::can::CANSocket::Ptr socket_;
        bool running_;

        struct
        {
            uint32_t axis_error = 0;
            uint8_t axis_state = 0;
            float iq_setpoint = 0.0f;
            float iq_measured = 0.0f;
            float fet_temperature = 0.0f;
            float motor_temperature = 0.0f;
            float bus_voltage = 0.0f;
            float bus_current = 0.0f;
            float pos_estimate = 0.0f;
            float vel_estimate = 0.0f;
            float torque_estimate = 0.0f;
        } info_[MAX_CAN_ID + 1];

        std::mutex mutex_;
    };

};

#endif