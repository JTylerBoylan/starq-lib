#include "starq/walking/walking_gait_controller.hpp"

namespace starq::walking
{

    WalkingGaitController::WalkingGaitController(starq::slam::Localization::Ptr localization)
        : localization_(localization),
          trajectory_file_reader_(std::make_shared<starq::TrajectoryFileReader>()),
          running_(false),
          sleep_duration_us_(10000)
    {
        if (!trajectory_file_reader_->load("/app/trajectories/walk.txt"))
        {
            std::cerr << "Failed to load walking trajectory file." << std::endl;
        }
    }

    WalkingGaitController::~WalkingGaitController()
    {
    }

    void WalkingGaitController::start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = true;
        std::thread(&WalkingGaitController::run, this).detach();
        /* TODO : Start MPC */
    }

    void WalkingGaitController::setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed)
    {
        desired_linear_speed_ = linear_speed;
        desired_angular_speed_ = angular_speed;
    }

    void WalkingGaitController::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        /* TODO : Stop MPC */
    }

    void WalkingGaitController::run()
    {
        while (true)
        {

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            /* TODO : Get COM Trajectory */
            /* TODO : Generate footfall positions */
            /* TODO : Update MPC */

            std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration_us_));
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }
    }
}