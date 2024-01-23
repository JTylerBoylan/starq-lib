#ifndef STARQ__LEG_COMMAND_PUBLISHER_HPP_
#define STARQ__LEG_COMMAND_PUBLISHER_HPP_

#include "starq/leg_controller.hpp"
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>

namespace starq
{

    /// @brief Leg command structure
    struct LegCommand
    {
        uint8_t leg_id = 0;
        uint32_t control_mode = 0;
        uint32_t input_mode = 0x1;
        VectorXf target_position = VectorXf();
        VectorXf target_velocity = VectorXf();
        VectorXf target_force = VectorXf();
    };

    class LegCommandPublisher
    {
    public:
        using Ptr = std::shared_ptr<LegCommandPublisher>;

        /// @brief Construct a new Leg Command Publisher object
        /// @param leg_controller Leg controller to publish commands to.
        LegCommandPublisher(LegController::Ptr leg_controller);

        /// @brief Destroy the Leg Command Publisher object
        ~LegCommandPublisher();

        /// @brief Send a leg command to the leg controller.
        /// @param leg_command Leg command to send.
        void sendCommand(const LegCommand &leg_command);

        /// @brief Clear the leg command publisher.
        void clear();

        /// @brief Get the leg controller.
        /// @return Leg controller.
        LegController::Ptr getLegController() { return leg_controller_; }

        /// @brief Set the stop on fail flag.
        /// @param stop_on_fail Stop on fail flag.
        void setStopOnFail(const bool stop_on_fail) { stop_on_fail_ = stop_on_fail; }

        /// @brief Set the sleep duration between commands.
        /// @param sleep_duration_us Sleep duration in microseconds.
        void setSleepDuration(const time_t sleep_duration_us) { sleep_duration_us_ = sleep_duration_us; }

    private:
        /// @brief Start the leg command publisher.
        /// @return If the leg command publisher was started successfully.
        bool start();

        /// @brief Stop the leg command publisher.
        /// @return If the leg command publisher was stopped successfully.
        bool stop();

        /// @brief Run the leg command publisher. (threaded)
        void run();

        LegController::Ptr leg_controller_;
        bool running_;
        bool stop_on_fail_;
        time_t sleep_duration_us_;
        std::mutex mutex_;
        std::unordered_map<uint8_t, LegCommand> leg_command_map_;
    };
}

#endif
