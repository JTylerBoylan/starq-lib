#ifndef STARQ_ODRIVE__ODRIVE_CONTROLLER_HPP_
#define STARQ_ODRIVE__ODRIVE_CONTROLLER_HPP_

#include "starq/odrive/can_driver.hpp"
#include "starq/odrive/can_listener.hpp"
#include "starq/motor_controller.hpp"

namespace starq::odrive
{

    class ODriveController : public starq::MotorController
    {
    public:
        using Ptr = std::shared_ptr<ODriveController>;

        /// @brief Connect to ODrive via CAN
        /// @param socket The CAN socket.
        ODriveController(const starq::can::CANSocket::Ptr socket);

        /// @brief Disconnect from ODrive.
        ~ODriveController();

        /// @brief Set the gear ratio
        /// @param motor_id The ID of the motor.
        /// @param gear_ratio The gear ratio of the motor.
        /// @return If the gear ratio was set successfully.
        bool setGearRatio(const uint8_t motor_id, const float gear_ratio) override;

        /// @brief Set the axis state.
        /// @param can_id The CAN ID of the ODrive.
        /// @param state The state to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setState(const uint8_t can_id, const uint32_t state) override;

        /// @brief Set the control mode.
        /// @param can_id The CAN ID of the ODrive.
        /// @param control_mode The control mode to set the ODrive to.
        /// @param input_mode The input mode to set the ODrive to. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1) override;

        /// @brief Set the position.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos The position to set the ODrive to. [rad]
        /// @param vel_ff The velocity feedforward to set the ODrive to. [rad/s] (default: 0)
        /// @param torque_ff The torque feedforward to set the ODrive to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) override;

        /// @brief Set the velocity.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel The velocity to set the ODrive to. [rad/s]
        /// @param torque_ff The torque feedforward to set the ODrive to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F) override;

        /// @brief Set the torque.
        /// @param can_id The CAN ID of the ODrive.
        /// @param torque The torque to set the ODrive to. [N-m]
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque) override;

        /// @brief Set the position gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos_gain The position gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain);

        /// @brief Set the velocity gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel_gain The velocity gain to set the ODrive to.
        /// @param vel_integrator_gain The velocity integrator gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain);

        /// @brief Set the velocity limit.
        /// @param can_id The CAN ID of the ODrive.
        /// @param velocity_limit The velocity limit to set the ODrive to.
        /// @param current_limit The current limit to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit);

        /// @brief Get the encoder position estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder position estimate in radians.
        float getPositionEstimate(const uint8_t can_id) override;

        /// @brief Get the encoder velocity estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder velocity estimate in radians per second.
        float getVelocityEstimate(const uint8_t can_id) override;

        /// @brief Get the controller torque estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The controller torque estimate in Newton-meters.
        float getTorqueEstimate(const uint8_t can_id) override;

        /// @brief Get the axis error.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The axis error.
        uint32_t getAxisError(const uint8_t can_id);

        /// @brief Get the axis state.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The axis state.
        uint8_t getAxisState(const uint8_t can_id);

        /// @brief Get the Iq setpoint.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The Iq setpoint in Amps.
        float getIqSetpoint(const uint8_t can_id);

        /// @brief Get the Iq measured.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The Iq measured in Amps.
        float getIqMeasured(const uint8_t can_id);

        /// @brief Get the FET temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The FET temperature in degrees Celsius.
        float getFETTemperature(const uint8_t can_id);

        /// @brief Get the motor temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The motor temperature in degrees Celsius.
        float getMotorTemperature(const uint8_t can_id);

        /// @brief Get the DC bus voltage.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus voltage in Volts.
        float getBusVoltage(const uint8_t can_id);

        /// @brief Get the DC bus current.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus current in Amps.
        float getBusCurrent(const uint8_t can_id);

        /// @brief Print the motor info.
        /// @param motor_id The ID of the motor.
        void printInfo(const uint8_t motor_id);

    private:
        ODriveCANDriver::Ptr driver_;
        ODriveCANListener::Ptr listener_;

        struct
        {
            uint32_t axis_state = 0;
            uint32_t control_mode = 0;
            uint32_t input_mode = 0;
            float pos_gain = 0.0f;
            float vel_gain = 0.0f;
            float integrator_gain = 0.0f;
            float velocity_limit = 0.0f;
            float current_limit = 0.0f;
            float gear_ratio = 1.0f;
        } configs_[MAX_MOTOR_ID + 1];
    };

}

#endif