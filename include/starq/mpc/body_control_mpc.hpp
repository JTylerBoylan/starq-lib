#ifndef STARQ_MPC__BODY_CONTROL_MPC_HPP_
#define STARQ_MPC__BODY_CONTROL_MPC_HPP_

#include "starq/mpc/mpc_types.hpp"
#include "starq/thread_runner.hpp"

#include <memory>

namespace starq::mpc
{

    class BodyControlMPC : public starq::ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<BodyControlMPC>;

        BodyControlMPC();

        ~BodyControlMPC();

        void setInertiaTensor(const Matrix3f &inertia_tensor);

        void setMass(const float &mass);

        void setGravity(const float &gravity);

    private:
        void run() override;

        Matrix3f inertia_tensor_;
        float mass_;
        float gravity_;

    };
}

#endif