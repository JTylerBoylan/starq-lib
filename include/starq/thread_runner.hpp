#ifndef STARQ__THREAD_RUNNER_HPP
#define STARQ__THREAD_RUNNER_HPP

#include <thread>
#include <mutex>

namespace starq
{
    class ThreadRunner
    {
    public:
        virtual bool start()
        {
            if (running_)
                return false;

            running_ = true;
            std::thread(&ThreadRunner::run, this).detach();
            return true;
        }

        virtual bool stop()
        {
            if (!running_)
                return false;

            running_ = false;
            return true;
        }

        bool isRunning() const
        {
            return running_;
        }

    private:
        virtual void run() = 0;

        bool running_ = false;
        std::mutex mutex_;
    };
}

#endif