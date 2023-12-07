#include "starq/odrive/can_listener.hpp"

namespace starq::odrive
{

    ODriveCANListener::ODriveCANListener(const starq::can::CANSocket::Ptr socket)
        : socket_(socket), running_(false)
    {
        start();
    }

    ODriveCANListener::~ODriveCANListener()
    {
        stop();
    }

    uint32_t ODriveCANListener::getAxisError(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].axis_error;
    }

    uint8_t ODriveCANListener::getAxisState(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].axis_state;
    }

    float ODriveCANListener::getIqSetpoint(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].iq_setpoint;
    }

    float ODriveCANListener::getIqMeasured(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].iq_measured;
    }

    float ODriveCANListener::getFETTemperature(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].fet_temperature;
    }

    float ODriveCANListener::getMotorTemperature(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].motor_temperature;
    }

    float ODriveCANListener::getBusVoltage(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].bus_voltage;
    }

    float ODriveCANListener::getBusCurrent(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].bus_current;
    }

    float ODriveCANListener::getPosEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].pos_estimate;
    }

    float ODriveCANListener::getVelEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].vel_estimate;
    }

    float ODriveCANListener::getTorqueEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].torque_estimate;
    }

    bool ODriveCANListener::start()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_)
        {
            std::cerr << "ODrive listener already running." << std::endl;
            return false;
        }

        running_ = true;
        poll_thread_ = std::thread(&ODriveCANListener::run, this);
        poll_thread_.detach();
        return true;
    }

    bool ODriveCANListener::stop()
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

    void ODriveCANListener::run()
    {
        bool running = true;
        while (running)
        {
            struct can_frame frame;
            if (socket_->receive(frame) < 0)
            {
                std::cerr << "Error listening on ODrive socket." << std::endl;
                continue;
            }

            uint8_t can_id = getCanID(frame.can_id);
            uint8_t cmd_id = getCommandID(frame.can_id);

            if (can_id > MAX_CAN_ID)
            {
                std::cerr << "Invalid CAN ID." << std::endl;
                continue;
            }

            switch (cmd_id)
            {
            case 0x001:
                // Heartbeat
                uint32_t axis_error;
                uint8_t axis_state;
                std::memcpy(&axis_error, frame.data, sizeof(axis_error));
                std::memcpy(&axis_state, frame.data + 4, sizeof(axis_state));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].axis_error = axis_error;
                    info_[can_id].axis_state = axis_state;
                }
                break;
            case 0x014:
                // Get_Iq
                float iq_setpoint, iq_measured;
                std::memcpy(&iq_setpoint, frame.data, sizeof(iq_setpoint));
                std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].iq_setpoint = iq_setpoint;
                    info_[can_id].iq_measured = iq_measured;
                }
                break;
            case 0x015:
                // Get_Temperature
                float fet_temperature, motor_temperature;
                std::memcpy(&fet_temperature, frame.data, sizeof(fet_temperature));
                std::memcpy(&motor_temperature, frame.data + 4, sizeof(motor_temperature));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].fet_temperature = fet_temperature;
                    info_[can_id].motor_temperature = motor_temperature;
                }
                break;
            case 0x017:
                // Get_Bus_Voltage_Current
                float bus_voltage, bus_current;
                std::memcpy(&bus_voltage, frame.data, sizeof(bus_voltage));
                std::memcpy(&bus_current, frame.data + 4, sizeof(bus_current));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].bus_voltage = bus_voltage;
                    info_[can_id].bus_current = bus_current;
                }
                break;
            case 0x009:
                // Get_Encoder_Estimates
                float pos_estimate, vel_estimate;
                std::memcpy(&pos_estimate, frame.data, sizeof(pos_estimate));
                std::memcpy(&vel_estimate, frame.data + 4, sizeof(vel_estimate));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].pos_estimate = pos_estimate;
                    info_[can_id].vel_estimate = vel_estimate;
                }
                break;
            case 0x01C:
                // Get_Torques
                float torque_estimate;
                std::memcpy(&torque_estimate, frame.data, sizeof(torque_estimate));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].torque_estimate = torque_estimate;
                }
                break;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                running = running_;
            }
        }
    }

}