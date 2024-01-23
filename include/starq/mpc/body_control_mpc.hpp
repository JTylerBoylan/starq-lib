#ifndef STARQ_MPC__BODY_CONTROL_MPC_HPP_
#define STARQ_MPC__BODY_CONTROL_MPC_HPP_

#include "starq/mpc/mpc_types.hpp"

#include <memory>
#include <thread>
#include <mutex>

namespace starq::mpc
{

    class BodyControlMPC
    {
    public:
        using Ptr = std::shared_ptr<BodyControlMPC>;

        BodyControlMPC();

        ~BodyControlMPC();

        void start();

        void stop();

        void setReferenceTrajectory(const CenterOfMassTrajectory &com_trajectory,
                                    const StrideTrajectory &stride_trajectory);

        void setInertiaTensor(const Matrix3f &inertia_tensor);

        void setMass(const float &mass);

        void setGravity(const float &gravity);

    private:
        void run();

        Matrix3f inertia_tensor_;
        float mass_;
        float gravity_;

        std::mutex mutex_;
    };
}

#endif