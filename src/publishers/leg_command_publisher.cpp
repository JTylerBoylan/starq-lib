#include "starq/publishers/leg_command_publisher.hpp"

namespace starq::publishers
{

    LegCommandPublisher::LegCommandPublisher(LegController::Ptr leg_controller)
        : leg_controller_(leg_controller),
          running_(false),
          stop_on_fail_(true),
          sleep_duration_us_(1000)
    {
        start();
    }

    LegCommandPublisher::~LegCommandPublisher()
    {
        if (running_)
            stop();
    }

    void LegCommandPublisher::push(LegCommand leg_cmd)
    {
        leg_cmd.stamp();
        std::lock_guard<std::mutex> lock(mutex_);
        leg_command_queue_.push(leg_cmd);
    }

    void LegCommandPublisher::push(std::vector<LegCommand> trajectory)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto &leg_cmd : trajectory)
        {
            leg_cmd.stamp();
            leg_command_queue_.push(leg_cmd);
        }
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
        using namespace std::chrono;

        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            bool has_command = false;
            LegCommand leg_cmd;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!leg_command_queue_.empty())
                {
                    leg_cmd = leg_command_queue_.top();
                    has_command = true;
                }
            }

            if (has_command && leg_cmd.release_time <= system_clock::now().time_since_epoch().count())
            {
                if (leg_controller_->getControlModeConfig(leg_cmd.leg_id) != leg_cmd.control_mode ||
                    leg_controller_->getInputModeConfig(leg_cmd.leg_id) != leg_cmd.input_mode)
                {
                    leg_controller_->setControlMode(leg_cmd.leg_id, leg_cmd.control_mode, leg_cmd.input_mode);
                }

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

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    leg_command_queue_.pop();
                }

                if (!command_success)
                {
                    std::cerr << "Failed to send leg command." << std::endl;

                    if (stop_on_fail_)
                    {
                        stop();
                    }
                }
            }

            std::this_thread::sleep_for(microseconds(sleep_duration_us_));
        }
    }
}
