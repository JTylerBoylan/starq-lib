#ifndef STARQ_ODRIVE__LISTENER_HPP_
#define STARQ_ODRIVE__LISTENER_HPP_

#include <thread>

#include "starq/can/socket.hpp"

#define MAX_CAN_ID 0x3F

namespace starq::odrive
{

    class ODriveListener
    {
    public:
        using Ptr = std::shared_ptr<ODriveListener>;

        ODriveListener(const starq::can::CANSocket::Ptr socket)
            : socket_(socket), running_(false)
        {
            start();
        }

        ~ODriveListener()
        {
            stop();
            poll_thread_.join();
        }

        uint32_t getAxisError(const uint8_t can_id) const
        {
            return info_[can_id].axis_error;
        }

        uint8_t getAxisState(const uint8_t can_id) const
        {
            return info_[can_id].axis_state;
        }

        float getIqSetpoint(const uint8_t can_id) const
        {
            return info_[can_id].iq_setpoint;
        }

        float getIqMeasured(const uint8_t can_id) const
        {
            return info_[can_id].iq_measured;
        }

        float getFETTemperature(const uint8_t can_id) const
        {
            return info_[can_id].fet_temperature;
        }

        float getMotorTemperature(const uint8_t can_id) const
        {
            return info_[can_id].motor_temperature;
        }

        float getBusVoltage(const uint8_t can_id) const
        {
            return info_[can_id].bus_voltage;
        }

        float getBusCurrent(const uint8_t can_id) const
        {
            return info_[can_id].bus_current;
        }

        float getTorqueTarget(const uint8_t can_id) const
        {
            return info_[can_id].torque_target;
        }

    private:
        bool start()
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (running_)
            {
                std::cerr << "ODrive listener already running." << std::endl;
                return false;
            }

            running_ = true;
            poll_thread_ = std::thread(&ODriveListener::run, this);
            poll_thread_.detach();
            return true;
        }

        bool stop()
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!running_)
            {
                std::cerr << "ODrive listener not running." << std::endl;
                return false;
            }

            running_ = false;
            return true;
        }
        
        void run()
        {
            while (running_)
            {
                struct can_frame frame;
                if (socket_->receive(frame) < 0)
                {
                    std::cerr << "Error listening on ODrive socket." << std::endl;
                    continue;
                }

                uint8_t can_id = getCanID(frame.can_id);
                uint8_t cmd_id = getCommandID(frame.can_id);

                std::lock_guard<std::mutex> lock(mutex_);

                switch (cmd_id)
                {
                case 0x001:
                    // Heartbeat
                    uint32_t axis_error;
                    uint8_t axis_state;
                    std::memcpy(&axis_error, frame.data, sizeof(axis_error));
                    std::memcpy(&axis_state, frame.data + 4, sizeof(axis_state));
                    info_[can_id].axis_error = axis_error;
                    info_[can_id].axis_state = axis_state;
                    break;
                case 0x014:
                    // Get_Iq
                    float iq_setpoint, iq_measured;
                    std::memcpy(&iq_setpoint, frame.data, sizeof(iq_setpoint));
                    std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));
                    info_[can_id].iq_setpoint = iq_setpoint;
                    info_[can_id].iq_measured = iq_measured;
                    break;
                case 0x015:
                    // Get_Temperature
                    float fet_temperature, motor_temperature;
                    std::memcpy(&fet_temperature, frame.data, sizeof(fet_temperature));
                    std::memcpy(&motor_temperature, frame.data + 4, sizeof(motor_temperature));
                    info_[can_id].fet_temperature = fet_temperature;
                    info_[can_id].motor_temperature = motor_temperature;
                    break;
                case 0x017:
                    // Get_Bus_Voltage_Current
                    float bus_voltage, bus_current;
                    std::memcpy(&bus_voltage, frame.data, sizeof(bus_voltage));
                    std::memcpy(&bus_current, frame.data + 4, sizeof(bus_current));
                    info_[can_id].bus_voltage = bus_voltage;
                    info_[can_id].bus_current = bus_current;
                    break;
                case 0x01C:
                    // Get_Torques
                    float torque_target;
                    std::memcpy(&torque_target, frame.data, sizeof(torque_target));
                    info_[can_id].torque_target = torque_target;
                    break;
                }
            }
        }

        uint8_t getCanID(const uint32_t arb_id) const
        {
            return (uint8_t)((arb_id >> 5) & 0b111111);
        }

        uint8_t getCommandID(const uint32_t arb_id) const
        {
            return (uint8_t)(arb_id & 0b11111);
        }

        const starq::can::CANSocket::Ptr socket_;
        bool running_;
        std::thread poll_thread_;

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
            float torque_target = 0.0f;
        } info_[MAX_CAN_ID];

        std::mutex mutex_;
    };

};

#endif