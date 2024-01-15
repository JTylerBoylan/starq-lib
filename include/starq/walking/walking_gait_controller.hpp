#ifndef STARQ_WALKING__WALKING_GAIT_CONTROLLER_HPP_
#define STARQ_WALKING__WALKING_GAIT_CONTROLLER_HPP_

#include "starq/gait_controller.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/slam/localization.hpp"

#include <thread>
#include <mutex>

namespace starq::walking
{

    class WalkingGaitController : public starq::GaitController
    {
    public:
        using Ptr = std::shared_ptr<WalkingGaitController>;

        /// @brief Create a walking gait controller.
        WalkingGaitController(starq::slam::Localization::Ptr localization);

        /// @brief Destroy the walking gait controller.
        ~WalkingGaitController();

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        std::string getGaitName() override { return "walk"; }

        /// @brief Start the gait.
        void start() override;

        /// @brief Stop the gait.
        void stop() override;

        /// @brief Set the walking speed.
        /// @param linear_speed Linear velocity of the walk.
        /// @param angular_speed Angular velocity of the walk.
        void setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed) override;

        /// @brief Set the sleep duration.
        /// @param sleep_duration_us Sleep duration in microseconds.
        void setSleepDuration(const time_t sleep_duration_us) { sleep_duration_us_ = sleep_duration_us; }

    private:
        /// @brief Run the gait.
        void run();

        bool running_;
        std::mutex mutex_;
        time_t sleep_duration_us_;

        Eigen::Vector3f desired_linear_speed_;
        Eigen::Vector3f desired_angular_speed_;

        starq::slam::Localization::Ptr localization_;
        starq::TrajectoryFileReader::Ptr trajectory_file_reader_;
    };

}

#endif