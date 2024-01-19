#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"

#define LEG_ID 0x0

#define MOTOR_ID_0 0x0
#define MOTOR_ID_1 0x1

#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

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
    printf("Created ODrive controller.\n");

    odrive->setGearRatio(MOTOR_ID_0, GEAR_RATIO_1);
    odrive->setGearRatio(MOTOR_ID_1, GEAR_RATIO_2);
    printf("Set ODrive gear ratios.\n");

    starq::LegController::Ptr leg = std::make_shared<LegController>(odrive);
    printf("Created leg controller.\n");

    leg->setMotorIDs(LEG_ID, {MOTOR_ID_0, MOTOR_ID_1});
    printf("Set motor IDs.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(
        LEG_LINK_1_LENGTH_M,
        LEG_LINK_2_LENGTH_M);
    leg->setLegDynamics(LEG_ID, fivebar_dynamics);
    printf("Set leg dynamics.\n");

    if (!leg->setState(LEG_ID, MotorState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis state.\n");

    if (!leg->setControlMode(LEG_ID, ControlMode::POSITION))
        return 1;
    printf("Set control mode.\n");

    const float center_x = 0.0f;
    const float center_y = -std::sqrt(2) * 0.1f;

    if (!leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y)))
        return 1;
    printf("Centered foot position.\n");

    const int revolutions = 10;
    const float radius = 0.025f;

    printf("Starting leg movement in sine wave.\n");
    for (float t = 0.0f; t <= revolutions * 2.0f * M_PI + 0.01; t += 0.01f)
    {
        const float x_off = radius * std::cos(t);
        const float y_off = radius * std::sin(t);

        VectorXf foot_position(2);
        foot_position << center_x + x_off, center_y + y_off;

        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));
        if (!leg->setFootPosition(LEG_ID, foot_position))
            return 1;

        usleep(2500);

        const VectorXf joint_angles = leg->getCurrentJointAngles(LEG_ID);
        printf("Joint angles: (%f, %f)\n", joint_angles(0), joint_angles(1));
    }

    if (!leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y)))
        return 1;
    printf("Centered foot position.\n");

    if (!leg->setState(LEG_ID, MotorState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}