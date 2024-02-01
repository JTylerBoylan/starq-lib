#ifndef STARQ_MPC__GAIT_SEQUENCER_HPP_
#define STARQ_MPC__GAIT_SEQUENCER_HPP_

#include <vector>
#include <map>
#include <chrono>
#include <string>
#include <memory>

#include "starq/thread_runner.hpp"

namespace starq::mpc
{

    struct Gait 
    {
        using Ptr = std::shared_ptr<Gait>;
        
        std::map<std::chrono::milliseconds, std::vector<bool>> sequence;

        bool load(const std::string& filename);
    };

    class GaitSequencer : public starq::ThreadRunner
    {
        public:
            using Ptr = std::shared_ptr<GaitSequencer>;

            GaitSequencer();

            void setGait(const Gait& gait);

        private:
            Gait current_gait_;
            Gait next_gait_;

            void run() override;


    };

}

#endif