#include "starq/leg_command_publisher.hpp"

#include <iostream>

namespace starq
{

    LegCommandPublisher::LegCommandPublisher(LegController::Ptr leg_controller)
        : leg_controller_(leg_controller),
          running_(false),
          stop_on_fail_(true),
          sleep_duration_us_(5000)
    {
        start();
    }

    LegCommandPublisher::~LegCommandPublisher()
    {
        if (running_)
            stop();
    }

    void LegCommandPublisher::sendCommand(const LegCommand &leg_command)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        leg_command_map_[leg_command.leg_id] = leg_command;
    }

    void LegCommandPublisher::clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        leg_command_map_.clear();
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
        using namespace std::chrono;

        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            for (auto iter = leg_command_map_.begin(); iter != leg_command_map_.end(); ++iter)
            {

                LegCommand leg_cmd;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    leg_cmd = iter->second;
                }

                leg_controller_->setControlMode(leg_cmd.leg_id, leg_cmd.control_mode, leg_cmd.input_mode);

                bool command_success = false;
                switch (leg_cmd.control_mode)
                {
                case ControlMode::POSITION:
                    command_success = leg_controller_->setFootPosition(leg_cmd.leg_id,
                                                                       leg_cmd.target_position,
                                                                       leg_cmd.target_velocity,
                                                                       leg_cmd.target_force);
                    break;
                case ControlMode::VELOCITY:
                    command_success = leg_controller_->setFootVelocity(leg_cmd.leg_id,
                                                                       leg_cmd.target_velocity,
                                                                       leg_cmd.target_force);
                    break;
                case ControlMode::TORQUE:
                    command_success = leg_controller_->setFootForce(leg_cmd.leg_id,
                                                                    leg_cmd.target_force);
                    break;
                }

                if (!command_success)
                {
                    std::cerr << "Failed to send leg command." << std::endl;

                    if (stop_on_fail_)
                    {
                        if (!stop())
                        {
                            std::cerr << "Failed to stop leg command publisher." << std::endl;
                        }
                    }
                }
            }

            std::this_thread::sleep_for(microseconds(sleep_duration_us_));
        }
    }
}
