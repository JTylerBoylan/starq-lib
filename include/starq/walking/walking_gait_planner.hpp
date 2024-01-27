#ifndef STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_
#define STARQ_WALKING__WALKING_GAIT_PLANNER_HPP_

#include "starq/slam/localization.hpp"
#include "starq/mpc/body_control_mpc.hpp"
#include "starq/thread_runner.hpp"

#include <thread>
#include <mutex>

namespace starq::walking
{

    class WalkingGaitPlanner : public starq::ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<WalkingGaitPlanner>;

        /// @brief Create a walking gait planner.
        WalkingGaitPlanner(starq::mpc::BodyControlMPC::Ptr body_control_mpc,
                           starq::slam::Localization::Ptr localization);

        /// @brief Destroy the walking gait planner.
        ~WalkingGaitPlanner();

        /// @brief Set the walking speed.
        /// @param linear_speed Linear velocity of the walk.
        /// @param angular_speed Angular velocity of the walk.
        void setDesiredVelocity(const Eigen::Vector3f &linear_speed, const Eigen::Vector3f &angular_speed);

        /// @brief Set the horizon time for the MPC.
        /// @param horizon_time Horizon time for the MPC.
        void setHorizonTime(const float &horizon_time) { horizon_time_ = horizon_time; }

        /// @brief Set the number of nodes for the MPC.
        /// @param node_count Number of nodes for the MPC.
        void setNodeCount(const int &node_count) { node_count_ = node_count; }

        /// @brief Set the hip locations for the MPC.
        /// @param hip_locations Hip locations for the MPC.
        void setHipLocations(const Eigen::Matrix<float, 3, NUMBER_OF_LEGS> &hip_locations) { hip_locations_ = hip_locations; }

        /// @brief Set the sleep duration for the walking gait planner.
        void setSleepDuration(const time_t &sleep_duration) { sleep_duration_ = sleep_duration; }

    private:
        /// @brief Run the walking gait planner.
        void run() override;

        starq::mpc::BodyControlMPC::Ptr body_control_mpc_;
        starq::slam::Localization::Ptr localization_;

        Eigen::Vector3f desired_linear_speed_;
        Eigen::Vector3f desired_angular_speed_;

        float horizon_time_;
        int node_count_;
        Eigen::Matrix<float, 3, NUMBER_OF_LEGS> hip_locations_;

        bool running_;
        time_t sleep_duration_;
        std::mutex mutex_;
    };

}

#endif