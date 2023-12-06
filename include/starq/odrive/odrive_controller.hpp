#ifndef STARQ_ODRIVE__ODRIVE_CONTROLLER_HPP_
#define STARQ_ODRIVE__ODRIVE_CONTROLLER_HPP_

#include "starq/odrive/can_driver.hpp"
#include "starq/odrive/can_listener.hpp"
#include "starq/controllers/motor_controller.hpp"

namespace starq::odrive
{

    class ODriveController : public starq::controllers::MotorController
    {
    public:
        using Ptr = std::shared_ptr<ODriveController>;

        /// @brief Connect to ODrive via CAN
        /// @param socket The CAN socket.
        ODriveController(const starq::can::CANSocket::Ptr socket);

        /// @brief Disconnect from ODrive.
        ~ODriveController();

        /// @brief Set the axis state.
        /// @param can_id The CAN ID of the ODrive.
        /// @param state The state to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t can_id, const uint32_t state) override;

        /// @brief Set the control mode.
        /// @param can_id The CAN ID of the ODrive.
        /// @param control_mode The control mode to set the ODrive to.
        /// @param input_mode The input mode to set the ODrive to. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1) override;

        /// @brief Set the position gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos_gain The position gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain) override;

        /// @brief Set the velocity gain.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel_gain The velocity gain to set the ODrive to.
        /// @param vel_integrator_gain The velocity integrator gain to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain) override;

        /// @brief Set the velocity limit.
        /// @param can_id The CAN ID of the ODrive.
        /// @param velocity_limit The velocity limit to set the ODrive to.
        /// @param current_limit The current limit to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit) override;

        /// @brief Set the position.
        /// @param can_id The CAN ID of the ODrive.
        /// @param pos The position to set the ODrive to.
        /// @param vel_ff The velocity feedforward to set the ODrive to. (default: 0)
        /// @param torque_ff The torque feedforward to set the ODrive to. (default: 0)
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) override;

        /// @brief Set the velocity.
        /// @param can_id The CAN ID of the ODrive.
        /// @param vel The velocity to set the ODrive to.
        /// @param torque_ff The torque feedforward to set the ODrive to. (default: 0)
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F) override;

        /// @brief Set the torque.
        /// @param can_id The CAN ID of the ODrive.
        /// @param torque The torque to set the ODrive to.
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque) override;

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
        /// @return The Iq setpoint.
        float getIqSetpoint(const uint8_t can_id);

        /// @brief Get the Iq measured.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The Iq measured.
        float getIqMeasured(const uint8_t can_id);

        /// @brief Get the FET temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The FET temperature.
        float getFETTemperature(const uint8_t can_id);

        /// @brief Get the motor temperature.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The motor temperature.
        float getMotorTemperature(const uint8_t can_id);

        /// @brief Get the DC bus voltage.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus voltage.
        float getBusVoltage(const uint8_t can_id);

        /// @brief Get the DC bus current.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The DC bus current.
        float getBusCurrent(const uint8_t can_id);

        /// @brief Get the encoder position estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder position estimate.
        float getPositionEstimate(const uint8_t can_id)  override;

        /// @brief Get the encoder velocity estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The encoder velocity estimate.
        float getVelocityEstimate(const uint8_t can_id)  override;

        /// @brief Get the controller torque estimate.
        /// @param can_id The CAN ID of the ODrive.
        /// @return The controller torque estimate.
        float getTorqueEstimate(const uint8_t can_id)  override;

    private:
        ODriveCANDriver::Ptr driver_;
        ODriveCANListener::Ptr listener_;
    };

}

#endif