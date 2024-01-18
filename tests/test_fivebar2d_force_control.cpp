#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"

#define LEG_ID 0x0

#define MOTOR_ID_0 0x0
#define MOTOR_ID_1 0x1

#define LEG_LINK_1_LENGTH_MM 50.0f
#define LEG_LINK_2_LENGTH_MM 150.0f

#define GEAR_RATIO_1 6.0f
#define GEAR_RATIO_2 6.0f

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;
using namespace starq::dynamics;

int main(void)
{
    printf("Hello world!\n");

    CANSocket::Ptr socket = std::make_shared<CANSocket>("can0");

    if (socket->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    ODriveController::Ptr odrive = std::make_shared<ODriveController>(socket);
    odrive->setGearRatio(MOTOR_ID_0, GEAR_RATIO_1);
    odrive->setGearRatio(MOTOR_ID_1, GEAR_RATIO_2);

    printf("Created ODrive controller.\n");

    LegController::Ptr leg = std::make_shared<LegController>(odrive);

    printf("Created leg controller.\n");

    leg->setMotorIDs(LEG_ID, {MOTOR_ID_0, MOTOR_ID_1});

    printf("Set motor IDs.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(
        LEG_LINK_1_LENGTH_MM,
        LEG_LINK_2_LENGTH_MM);
    leg->setLegDynamics(LEG_ID, fivebar_dynamics);

    printf("Set leg dynamics.\n");

    if (!leg->setState(LEG_ID, MotorState::CLOSED_LOOP_CONTROL))
        return 1;

    printf("Set axis state to closed loop control.\n");

    if (!leg->setControlMode(LEG_ID, ControlMode::TORQUE))
        return 1;

    printf("Set to force control mode.\n");

    const float force_x = 0.0f;
    const float force_y = -10.0f;
    printf("Applying Force: %f, %f\n", force_x, force_y);

    const VectorXf current_joint_angles = leg->getCurrentJointAngles(LEG_ID);

    MatrixXf jacobian;
    fivebar_dynamics->getJacobian(current_joint_angles, jacobian);
    const VectorXf joint_torque = jacobian.transpose() * Vector2f(force_x, force_y);

    printf("Joint torque: %f, %f\n", joint_torque(0), joint_torque(1));

    if (!leg->setFootForce(LEG_ID, Vector2f(force_x, force_y)))
        return 1;

    sleep(10);

    printf("\n");

    if (!leg->setState(LEG_ID, MotorState::IDLE))
        return 1;

    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}