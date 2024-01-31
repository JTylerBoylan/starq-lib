#ifndef STARQ_MPC__GAIT_SEQUENCE_HPP_
#define STARQ_MPC__GAIT_SEQUENCE_HPP_

#include <memory>
#include <vector>
#include <map>
#include <chrono>

#define GAIT_SEQUENCE_RESOLUTION 1000

namespace starq::mpc
{
    using StanceState = std::vector<bool>;

    class GaitSequence
    {
    public:
        using Ptr = std::shared_ptr<GaitSequence>;

        GaitSequence()
            : time_start_(0), time_repeat_(0), time_end_(0)
        {
        }

        ~GaitSequence()
        {
        }

        std::map<time_t, StanceState> &getSequence()
        {
            return sequence_;
        }

        void load(const std::string &file_path);

        void append(GaitSequence &gait_sequence)
        {
            for (auto it = gait_sequence.getSequence().begin(); it != gait_sequence.getSequence().end(); ++it)
            {
                const time_t seq_time_ms = time_end_ + it->first;
                sequence_.insert(std::make_pair(seq_time_ms, it->second));
            }
            time_repeat_ = time_end_;
            time_end_ += GAIT_SEQUENCE_RESOLUTION;
        }

        void getStanceState(const time_t &time_ms)
        {
            const time_t time_ms_bounded = getTimeInBounds(time_ms);
            const auto it = sequence_.lower_bound(time_ms_bounded);
        }

        void clearUntil(const time_t &time_ms)
        {
            const time_t time_ms_bounded = getTimeInBounds(time_ms);
            auto it = sequence_.lower_bound(time_ms_bounded);

            if (time_repeat_ < time_ms_bounded)
            {
                auto repeat_it = sequence_.lower_bound(time_repeat_);
                if (repeat_it != sequence_.begin())
                {
                    it = --repeat_it;
                }
                else
                {
                    it = sequence_.begin();
                }
            }

            sequence_.erase(sequence_.begin(), it);
        }

    private:
        time_t getTimeInBounds(const time_t &time_ms)
        {
            if (time_ms < time_start_)
            {
                return time_start_;
            }
            else if (time_ms > time_end_)
            {
                const time_t overshoot = time_ms - time_end_;
                return time_repeat_ + (overshoot % (time_end_ - time_repeat_));
            }
            else
            {
                return time_ms;
            }
        }

        time_t time_start_;
        time_t time_repeat_;
        time_t time_end_;
        std::map<time_t, StanceState> sequence_;
    };

}

#endif