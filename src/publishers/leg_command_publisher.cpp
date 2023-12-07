#include "starq/publishers/leg_command_publisher.hpp"

namespace starq::publishers
{

    LegCommandPublisher::LegCommandPublisher(LegController::Ptr leg_controller)
        : leg_controller_(leg_controller), running_(false)
    {
        start();
    }

    LegCommandPublisher::~LegCommandPublisher()
    {
        stop();
    }

    void LegCommandPublisher::insert(LegCommand leg_cmd)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        leg_command_queue_.push(leg_cmd);
    }

    void LegCommandPublisher::clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!leg_command_queue_.empty())
        {
            leg_command_queue_.pop();
        }
    }

    bool LegCommandPublisher::start()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_)
        {
            std::cerr << "Leg command publisher is already running." << std::endl;
            return false;
        }

        running_ = true;
        std::thread(&LegCommandPublisher::run, this).detach();
        return true;
    }

    bool LegCommandPublisher::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!running_)
        {
            std::cerr << "Leg command publisher is not running." << std::endl;
            return false;
        }

        running_ = false;
        return true;
    }

    void LegCommandPublisher::run()
    {
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            LegCommand leg_cmd;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (leg_command_queue_.empty())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                leg_cmd = leg_command_queue_.top();
            }

            if (leg_cmd.release_time > std::chrono::system_clock::now().time_since_epoch().count())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (leg_controller_->getControlModeConfig(leg_cmd.leg_id) != leg_cmd.control_mode ||
                leg_controller_->getInputModeConfig(leg_cmd.leg_id) != leg_cmd.input_mode)
                leg_controller_->setControlMode(leg_cmd.leg_id, leg_cmd.control_mode, leg_cmd.input_mode);

            switch (leg_cmd.control_mode)
            {
            case ControlMode::POSITION:
                leg_controller_->setFootPosition(leg_cmd.leg_id, leg_cmd.target_position, leg_cmd.target_velocity, leg_cmd.target_force);
                break;
            case ControlMode::VELOCITY:
                leg_controller_->setFootVelocity(leg_cmd.leg_id, leg_cmd.target_velocity, leg_cmd.target_force);
                break;
            case ControlMode::TORQUE:
                leg_controller_->setFootForce(leg_cmd.leg_id, leg_cmd.target_force);
                break;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                leg_command_queue_.pop();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

}