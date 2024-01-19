#ifndef STARQ_TESTING__DUMMY_MOTOR_CONTROLLER_HPP_
#define STARQ_TESTING__DUMMY_MOTOR_CONTROLLER_HPP_

#include "starq/motor_controller.hpp"

#include <iostream>

namespace starq::testing
{

    /// @brief A dummy motor controller that does nothing. For testing.
    class DummyMotorController : public starq::MotorController
    {
    public:
        DummyMotorController() {}

        ~DummyMotorController() {}

        bool setGearRatio(const uint8_t motor_id, const float gear_ratio) override
        {
            std::cout << "Setting gear ratio for motor " << (int)motor_id << " to " << gear_ratio << std::endl;
            return true;
        }

        bool setState(const uint8_t motor_id, const uint32_t state) override
        {
            std::cout << "Setting state for motor " << (int)motor_id << " to " << state << std::endl;
            return true;
        }

        bool setControlMode(const uint8_t motor_id, const uint32_t control_mode, const uint32_t input_mode = 0x1) override
        {
            std::cout << "Setting control mode for motor " << (int)motor_id << " to " << control_mode << " with input mode " << input_mode << std::endl;
            return true;
        }

        bool setPosition(const uint8_t motor_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) override
        {
            std::cout << "Setting position for motor " << (int)motor_id << " to " << pos
                      << " with velocity feedforward " << vel_ff
                      << " and torque feedforward " << torque_ff << std::endl;

            last_pos_[motor_id] = pos;
            last_vel_[motor_id] = vel_ff;
            last_torque_[motor_id] = torque_ff;
            return true;
        }

        bool setVelocity(const uint8_t motor_id, const float vel, const float torque_ff = 0.F) override
        {
            std::cout << "Setting velocity for motor " << (int)motor_id << " to " << vel
                      << " with torque feedforward " << torque_ff << std::endl;

            last_vel_[motor_id] = vel;
            last_torque_[motor_id] = torque_ff;
            return true;
        }

        bool setTorque(const uint8_t motor_id, const float torque) override
        {
            std::cout << "Setting torque for motor " << (int)motor_id << " to " << torque << std::endl;

            last_torque_[motor_id] = torque;
            return true;
        }

        float getPositionEstimate(const uint8_t motor_id) override
        {
            return last_pos_[motor_id];
        }

        float getVelocityEstimate(const uint8_t motor_id) override
        {
            return last_vel_[motor_id];
        }

        float getTorqueEstimate(const uint8_t motor_id) override
        {
            return last_torque_[motor_id];
        }

    private:
        float last_pos_[MAX_MOTOR_ID + 1];
        float last_vel_[MAX_MOTOR_ID + 1];
        float last_torque_[MAX_MOTOR_ID + 1];
    };

}

#endif