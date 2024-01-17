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

    leg->setState(LEG_ID, MotorState::CLOSED_LOOP_CONTROL);

    printf("Set axis state to closed loop control.\n");

    leg->setControlMode(LEG_ID, ControlMode::POSITION);

    printf("Set to position control mode.\n");

    const float center_x = 0.0f;
    const float center_y = -std::sqrt(2) * 100;

    leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y));

    printf("Set foot position to (%f, %f)\n", center_x, center_y);

    sleep(1);

    printf("\n");

    leg->setControlMode(LEG_ID, ControlMode::TORQUE);

    printf("Set to force control mode.\n");

    const float force = 10.0f;
    printf("Appling constant downward force of %f N for 10 seconds.\n", force);

    leg->setFootForce(LEG_ID, Vector2f(0.0f, force));

    sleep(10);

    printf("\n");

    leg->setControlMode(LEG_ID, ControlMode::POSITION);

    printf("Set to position control mode.\n");

    leg->setFootPosition(LEG_ID, Vector2f(center_x, center_y));

    printf("Set foot position to (%f, %f)\n", center_x, center_y);

    sleep(1);

    leg->setState(LEG_ID, MotorState::IDLE);

    printf("Set axis state to idle.\n");

    printf("Done.\n");

    return 0;
}