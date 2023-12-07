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
        time_t release_time;
        uint8_t leg_id;
        uint32_t control_mode;
        uint32_t input_mode = 0x1;
        VectorXf target_position = VectorXf();
        VectorXf target_velocity = VectorXf();
        VectorXf target_force = VectorXf();
    };

    class LegCommandPublisher
    {
    public:
        using Ptr = std::shared_ptr<LegCommandPublisher>;

        LegCommandPublisher(LegController::Ptr leg_controller);

        ~LegCommandPublisher();

        void insert(LegCommand leg_cmd);

        void clear();

    private:
        bool start();

        bool stop();

        void run();

        struct CompareLegCommand
        {
            bool operator()(const LegCommand &lhs, const LegCommand &rhs) const
            {
                return lhs.release_time > rhs.release_time;
            }
        };

        LegController::Ptr leg_controller_;
        bool running_;
        std::priority_queue<LegCommand, std::vector<LegCommand>, CompareLegCommand> leg_command_queue_;
        std::mutex mutex_;
    };
}

#endif
