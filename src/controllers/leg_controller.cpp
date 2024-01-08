#include "starq/controllers/leg_controller.hpp"
#include <iostream>

namespace starq::controllers
{

    LegController::LegController(starq::controllers::MotorController::Ptr motor_controller)
        : motor_controller_(motor_controller)
    {
    }

    LegController::~LegController()
    {
    }

    void LegController::setMotorIDs(const uint8_t leg_id, const std::vector<uint32_t> motor_ids)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return;
        }

        configs_[leg_id].motor_ids = motor_ids;
    }

    void LegController::setLegDynamics(const uint8_t leg_id, const starq::dynamics::LegDynamics::Ptr dynamics)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return;
        }

        configs_[leg_id].dynamics = dynamics;
    }

    bool LegController::setAxisState(const uint8_t leg_id, const uint32_t state)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        bool success = true;
        for (const auto &motor_id : configs_[leg_id].motor_ids)
        {
            success &= motor_controller_->setAxisState(motor_id, state);
        }

        configs_[leg_id].axis_state = state;
        return success;
    }

    bool LegController::setControlMode(const uint8_t leg_id, const uint32_t control_mode, const uint32_t input_mode)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        bool success = true;
        for (const auto &motor_id : configs_[leg_id].motor_ids)
        {
            success &= motor_controller_->setControlMode(motor_id, control_mode, input_mode);
        }

        configs_[leg_id].control_mode = control_mode;
        configs_[leg_id].input_mode = input_mode;
        return success;
    }

    bool LegController::setPosGain(const uint8_t leg_id, const float pos_gain)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        bool success = true;
        for (const auto &motor_id : configs_[leg_id].motor_ids)
        {
            success &= motor_controller_->setPosGain(motor_id, pos_gain);
        }

        configs_[leg_id].pos_gain = pos_gain;
        return success;
    }

    bool LegController::setVelGains(const uint8_t leg_id, const float vel_gain, const float vel_integrator_gain)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        bool success = true;
        for (const auto &motor_id : configs_[leg_id].motor_ids)
        {
            success &= motor_controller_->setVelGains(motor_id, vel_gain, vel_integrator_gain);
        }

        configs_[leg_id].vel_gain = vel_gain;
        configs_[leg_id].vel_integrator_gain = vel_integrator_gain;
        return success;
    }

    bool LegController::setLimits(const uint8_t leg_id, const float vel_limit, const float current_limit)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        bool success = true;
        for (const auto &motor_id : configs_[leg_id].motor_ids)
        {
            success &= motor_controller_->setLimits(motor_id, vel_limit, current_limit);
        }

        configs_[leg_id].velocity_limit = vel_limit;
        configs_[leg_id].current_limit = current_limit;
        return success;
    }

    bool LegController::setFootPosition(const uint8_t leg_id,
                                        const VectorXf &foot_position,
                                        const VectorXf &foot_velocity_ff,
                                        const VectorXf &foot_torque_ff)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        if (foot_position.size() == 0)
        {
            std::cerr << "Empty foot position." << std::endl;
            return false;
        }

        const bool has_velocity_ff = foot_velocity_ff.size() > 0;
        const bool has_torque_ff = foot_torque_ff.size() > 0;

        MatrixXf jacobian;
        if (has_velocity_ff || has_torque_ff)
        {
            const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

            if (!configs_[leg_id].dynamics->getJacobian(current_joint_angles, jacobian))
            {
                std::cerr << "Failed to get Jacobian." << std::endl;
                return false;
            }
        }

        VectorXf joint_angles;
        if (!configs_[leg_id].dynamics->getInverseKinematics(foot_position, joint_angles))
        {
            std::cerr << "Failed to get inverse kinematics." << std::endl;
            return false;
        }

        VectorXf joint_velocity;
        if (has_velocity_ff)
        {
            if (jacobian.rows() != foot_velocity_ff.size())
            {
                std::cerr << "Invalid foot velocity feedforward." << std::endl;
                return false;
            }

            joint_velocity = jacobian.inverse() * foot_velocity_ff;
        }
        else
        {
            joint_velocity = VectorXf::Zero(joint_angles.size());
        }

        VectorXf joint_torque;
        if (has_torque_ff)
        {
            if (jacobian.rows() != foot_torque_ff.size())
            {
                std::cerr << "Invalid foot torque feedforward." << std::endl;
                return false;
            }

            joint_torque = jacobian.transpose() * foot_torque_ff;
        }
        else
        {
            joint_torque = VectorXf::Zero(joint_angles.size());
        }

        return setJointAngles(leg_id, joint_angles, joint_velocity, joint_torque);
    }

    bool LegController::setFootVelocity(const uint8_t leg_id,
                                        const VectorXf &foot_velocity,
                                        const VectorXf &foot_torque_ff)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        if (foot_velocity.size() == 0)
        {
            std::cerr << "Empty foot velocity." << std::endl;
            return false;
        }

        const bool has_torque_ff = foot_torque_ff.size() > 0;

        if (has_torque_ff && foot_torque_ff.size() != foot_velocity.size())
        {
            std::cerr << "Invalid foot torque feedforward." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

        MatrixXf jacobian;
        if (!configs_[leg_id].dynamics->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        if (jacobian.rows() != foot_velocity.size())
        {
            std::cerr << "Invalid foot velocity." << std::endl;
            return false;
        }

        VectorXf joint_velocities = jacobian.inverse() * foot_velocity;

        VectorXf joint_torque;
        if (has_torque_ff)
        {
            if (jacobian.rows() != foot_torque_ff.size())
            {
                std::cerr << "Invalid foot torque feedforward." << std::endl;
                return false;
            }

            joint_torque = jacobian.transpose() * foot_torque_ff;
        }
        else
        {
            joint_torque = VectorXf::Zero(joint_velocities.size());
        }

        return setJointVelocities(leg_id, joint_velocities, joint_torque);
    }

    bool LegController::setFootForce(const uint8_t leg_id,
                                     const VectorXf &foot_force)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        if (foot_force.size() == 0)
        {
            std::cerr << "Empty foot force." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

        MatrixXf jacobian;
        if (!configs_[leg_id].dynamics->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        if (jacobian.rows() != foot_force.size())
        {
            std::cerr << "Invalid foot force." << std::endl;
            return false;
        }

        const VectorXf joint_torque = jacobian.transpose() * foot_force;

        return setJointTorques(leg_id, joint_torque);
    }

    bool LegController::getFootPositionEstimate(const uint8_t leg_id,
                                                VectorXf &foot_position)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

        if (!configs_[leg_id].dynamics->getForwardKinematics(current_joint_angles, foot_position))
        {
            std::cerr << "Failed to get forward kinematics." << std::endl;
            return false;
        }

        return true;
    }

    bool LegController::getFootVelocityEstimate(const uint8_t leg_id,
                                                VectorXf &foot_velocity)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

        MatrixXf jacobian;
        if (!configs_[leg_id].dynamics->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        VectorXf joint_velocities = getCurrentJointVelocities(leg_id);
        foot_velocity = jacobian * joint_velocities;

        return true;
    }

    bool LegController::getFootForceEstimate(const uint8_t leg_id,
                                             VectorXf &foot_force)
    {

        if (leg_id > MAX_LEG_ID)
        {
            std::cerr << "Invalid leg ID." << std::endl;
            return false;
        }

        if (!configs_[leg_id].dynamics)
        {
            std::cerr << "Leg dynamics not set." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles(leg_id);

        MatrixXf jacobian;
        if (!configs_[leg_id].dynamics->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        VectorXf joint_torques = getCurrentJointTorques(leg_id);
        foot_force = jacobian * joint_torques;

        return true;
    }

    VectorXf LegController::getCurrentJointAngles(const uint8_t leg_id)
    {
        VectorXf joint_angles(configs_[leg_id].motor_ids.size());
        for (size_t i = 0; i < configs_[leg_id].motor_ids.size(); i++)
        {
            joint_angles(i) = motor_controller_->getPositionEstimate(configs_[leg_id].motor_ids[i]);
        }
        return joint_angles;
    }

    VectorXf LegController::getCurrentJointVelocities(const uint8_t leg_id)
    {
        VectorXf joint_velocities(configs_[leg_id].motor_ids.size());
        for (size_t i = 0; i < configs_[leg_id].motor_ids.size(); i++)
        {
            joint_velocities(i) = motor_controller_->getVelocityEstimate(configs_[leg_id].motor_ids[i]);
        }
        return joint_velocities;
    }

    VectorXf LegController::getCurrentJointTorques(const uint8_t leg_id)
    {
        VectorXf joint_torques(configs_[leg_id].motor_ids.size());
        for (size_t i = 0; i < configs_[leg_id].motor_ids.size(); i++)
        {
            joint_torques(i) = motor_controller_->getTorqueEstimate(configs_[leg_id].motor_ids[i]);
        }
        return joint_torques;
    }

    bool LegController::setJointAngles(const uint8_t leg_id,
                                       const VectorXf &joint_angles,
                                       const VectorXf &joint_velocity_ff,
                                       const VectorXf &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < configs_[leg_id].motor_ids.size()) && success; i++)
        {
            success &= motor_controller_->setPosition(configs_[leg_id].motor_ids[i],
                                                      joint_angles(i, 0),
                                                      joint_velocity_ff(i, 0),
                                                      joint_torque_ff(i, 0));
        }
        return success;
    }

    bool LegController::setJointVelocities(const uint8_t leg_id,
                                           const VectorXf &joint_velocities,
                                           const VectorXf &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < configs_[leg_id].motor_ids.size()) && success; i++)
        {
            success &= motor_controller_->setVelocity(configs_[leg_id].motor_ids[i],
                                                      joint_velocities(i, 0),
                                                      joint_torque_ff(i, 0));
        }
        return success;
    }

    bool LegController::setJointTorques(const uint8_t leg_id,
                                        const VectorXf &joint_torques)
    {
        bool success = true;
        for (size_t i = 0; (i < configs_[leg_id].motor_ids.size()) && success; i++)
        {
            success &= motor_controller_->setTorque(configs_[leg_id].motor_ids[i],
                                                    joint_torques(i, 0));
        }
        return success;
    }

}