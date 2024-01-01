#include "starq/walking/walking_gait_controller.hpp"

namespace starq::walking
{

    WalkingGaitController::WalkingGaitController(starq::slam::Localization::Ptr localization)
        : localization_(localization),
          trajectory_file_reader_(std::make_shared<starq::trajectories::TrajectoryFileReader>()),
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

    void WalkingGaitController::moveTo(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation)
    {
        desired_position_ = position;
        desired_orientation_ = orientation;

        /* TODO : Start MPC */

        if (!running_)
        {
            running_ = true;
            std::thread(&WalkingGaitController::run, this).detach();
        }
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

            /* TODO : Check if at desired state */

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