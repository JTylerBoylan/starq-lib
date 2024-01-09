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

    printf("Created ODrive controller.\n");

    LegController::Ptr leg = std::make_shared<LegController>(odrive);

    printf("Created leg controller.\n");

    leg->setMotorIDs(LEG_ID, {MOTOR_ID_0, MOTOR_ID_1});

    printf("Set motor IDs.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(
        LEG_LINK_1_LENGTH_MM,
        LEG_LINK_2_LENGTH_MM,
        GEAR_RATIO_1,
        GEAR_RATIO_2);
    leg->setLegDynamics(LEG_ID, fivebar_dynamics);

    printf("Set leg dynamics.\n");

    leg->setState(LEG_ID, MotorState::CLOSED_LOOP_CONTROL);

    printf("Set axis state.\n");

    leg->setControlMode(LEG_ID, ControlMode::POSITION);

    printf("Set control mode.\n");

    const float center_x = 0.0f;
    const float center_y = -150.0f;

    leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y));

    printf("Starting leg movement in sine wave.\n");
    for (float t = 0.0f; t <= 2.0f * M_PI + 0.01; t += 0.01f)
    {
        const float y_off = 25.0f * std::sin(t);

        VectorXf foot_position(2);
        foot_position << 0, center_y + y_off;

        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));

        leg->setFootPosition(LEG_ID, foot_position);
        usleep(5000);
    }

    leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y));
    leg->setState(LEG_ID, MotorState::IDLE);

    return 0;
}