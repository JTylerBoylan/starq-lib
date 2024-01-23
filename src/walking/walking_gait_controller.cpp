#include "starq/walking/walking_gait_controller.hpp"

namespace starq::walking
{

    WalkingGaitController::WalkingGaitController(starq::LegCommandPublisher::Ptr leg_command_publisher,
                                                 starq::slam::Localization::Ptr localization)
        : localization_(localization),
          leg_command_publisher_(leg_command_publisher),
          body_control_mpc_(std::make_shared<starq::mpc::BodyControlMPC>()),
          walking_gait_planner_(std::make_shared<WalkingGaitPlanner>(body_control_mpc_, localization_))
    {
    }

    WalkingGaitController::~WalkingGaitController()
    {
    }

    void WalkingGaitController::start()
    {
        walking_gait_planner_->start();
        body_control_mpc_->start();
    }

    void WalkingGaitController::stop()
    {
        walking_gait_planner_->stop();
        body_control_mpc_->stop();
    }

    void WalkingGaitController::setVelocity(const Vector3f &linear_speed, const Vector3f &angular_speed)
    {
        walking_gait_planner_->setDesiredVelocity(linear_speed, angular_speed);
    }
}