#ifndef STARQ_PUBLISHERS__LEG_COMMAND_PUBLISHER_HPP_
#define STARQ_PUBLISHERS__LEG_COMMAND_PUBLISHER_HPP_

#include "starq/leg_controller.hpp"
#include <queue>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>

namespace starq
{

    /// @brief Leg command structure
    struct LegCommand
    {
        double delay_in_seconds = 0.0;
        uint8_t leg_id = 0;
        uint32_t control_mode = 0;
        uint32_t input_mode = 0x1;
        VectorXf target_position = VectorXf();
        VectorXf target_velocity = VectorXf();
        VectorXf target_force = VectorXf();

        time_t release_time = 0L;

        /// @brief Stamp the release time of the leg command.
        void stamp()
        {
            release_time = std::chrono::system_clock::now().time_since_epoch().count() + delay_in_seconds * 1000;
        }  
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

        /// @brief Add a leg command to the queue.
        /// @param leg_cmd Command to add to the queue.
        void push(LegCommand leg_cmd);

        /// @brief Add a vector of leg commands to the queue.
        /// @param trajectory Vector of leg commands to add to the queue.
        void push(std::vector<LegCommand> trajectory);

        /// @brief Clear the leg command queue.
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

        /// @brief Sort leg commands by release time.
        struct CompareLegCommand
        {
            bool operator()(const LegCommand &lhs, const LegCommand &rhs) const
            {
                return lhs.release_time > rhs.release_time;
            }
        };

        LegController::Ptr leg_controller_;
        bool running_;
        bool stop_on_fail_;
        time_t sleep_duration_us_;
        std::priority_queue<LegCommand, std::vector<LegCommand>, CompareLegCommand> leg_command_queue_;
        std::mutex mutex_;
    };
}

#endif
