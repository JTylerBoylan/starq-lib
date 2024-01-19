#include "starq/walking/walking_gait_controller.hpp"

namespace starq::walking
{

    WalkingGaitController::WalkingGaitController(starq::slam::Localization::Ptr localization)
        : localization_(localization),
          trajectory_file_reader_(std::make_shared<starq::TrajectoryFileReader>()),
          running_(false),
          stride_length_(0.1f),
          nodes_per_stride_(5),
          lookahead_strides_(4)
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

    void WalkingGaitController::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        /* TODO : Stop MPC */
    }

    void WalkingGaitController::setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed)
    {
        desired_linear_speed_ = linear_speed;
        desired_angular_speed_ = angular_speed;
    }

    void WalkingGaitController::run()
    {
        const int nodes_per_stride = nodes_per_stride_;
        const int num_nodes = nodes_per_stride * lookahead_strides_;
        const float stride_frequency = desired_linear_speed_.norm() / stride_length_;
        const float pause_duration = 1.0f / (stride_frequency * nodes_per_stride);

        while (true)
        {

            /* TODO : Update swing/stance states */

            /* TODO : Send swing leg trajectory */

            for (int n = 0; n < nodes_per_stride_; n++)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!running_)
                        return;
                }

                /* TODO : Get COM Trajectory */
                /* TODO : Generate footfall positions */
                /* TODO : Get foot positions over the trajectory */
                /* TODO : Update MPC */

                std::this_thread::sleep_for(std::chrono::microseconds((time_t)(pause_duration * 1E6)));
            }
        }
    }
}