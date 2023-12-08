#ifndef STARQ_PUBLISHERS__LEG_COMMAND_PUBLISHER_HPP_
#define STARQ_PUBLISHERS__LEG_COMMAND_PUBLISHER_HPP_

#include "starq/controllers/leg_controller.hpp"
#include <queue>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>

namespace starq::publishers
{
    using namespace starq::controllers;

    /// @brief Leg command structure
    struct LegCommand
    {
        using Ptr = std::shared_ptr<LegCommand>;

        time_t release_time;
        uint8_t leg_id;
        uint32_t control_mode = 0;
        uint32_t input_mode = 0x1;
        VectorXf target_position = VectorXf();
        VectorXf target_velocity = VectorXf();
        VectorXf target_force = VectorXf();

        /// @brief Construct a new Leg Command object
        /// @param leg_id ID of the leg.
        LegCommand(const uint8_t leg_id);

        /// @brief Set the time from now to release the command.
        /// @param time_in_seconds Time to release in seconds.
        void setTimeFromNow(const double time_in_seconds);
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
        void push(LegCommand::Ptr leg_cmd);

        /// @brief Clear the leg command queue.
        void clear();

        /// @brief Get the leg controller.
        /// @return Leg controller.
        LegController::Ptr getLegController() { return leg_controller_; }

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
            bool operator()(const LegCommand::Ptr &lhs, const LegCommand::Ptr &rhs) const
            {
                return lhs->release_time > rhs->release_time;
            }
        };

        LegController::Ptr leg_controller_;
        bool running_;
        std::priority_queue<LegCommand::Ptr, std::vector<LegCommand::Ptr>, CompareLegCommand> leg_command_queue_;
        std::mutex mutex_;
    };
}

#endif
