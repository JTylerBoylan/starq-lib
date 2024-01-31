#ifndef STARQ_MPC__MPC_PLANNER_HPP_
#define STARQ_MPC__MPC_PLANNER_HPP_

#include "starq/mpc/mpc_types.hpp"

#include <memory>
#include <chrono>

#define GAIT_SEQUENCE_RESOLUTION 1000

namespace starq::mpc
{

    class MPCPlanner 
    {
    public:
        using Ptr = std::shared_ptr<MPCPlanner>;

        MPCPlanner();

        ~MPCPlanner();

        void setDesiredVelocity(const Eigen::Vector3f &linear_velocity, const Eigen::Vector3f &angular_velocity)
        {
            linear_velocity_ = linear_velocity;
            angular_velocity_ = angular_velocity;
        }

        void setStrideLength(const float &stride_length)
        {
            stride_length_ = stride_length;
        }

        void loadGaitSequence(const std::string &file_path);

    private:

        StanceState getStanceState(const time_t &time_ms)
        {
            const time_t stride_duration_ms = stride_length_ / linear_velocity_.norm() * 1E3;
            const time_t stride_time_ms = time_ms % stride_duration_ms;
            const time_t stride_time_ms_norm = stride_time_ms * GAIT_SEQUENCE_RESOLUTION / stride_duration_ms;
            return gait_sequence_.lower_bound(stride_time_ms_norm)->second;
        }

        Eigen::Vector3f linear_velocity_;
        Eigen::Vector3f angular_velocity_;

        time_t start_time_;
        GaitSequence gait_sequence_;

        float stride_length_;

    };

}

#endif