#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

#define CAN_ID 0

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

    odrive->setState(CAN_ID, MotorState::CLOSED_LOOP_CONTROL);
    odrive->setControlMode(CAN_ID, ControlMode::POSITION);
    odrive->setPosition(CAN_ID, 0.0f);
    sleep(1);
    printf("--------------------\n");
    odrive->printInfo(CAN_ID);

    const float position_increment = 0.2;
    for (float p = 0.0f; p <= 1.0f; p += position_increment)
    {
        printf("--------------------\n");
        printf("Setting position to %f\n", p);
        odrive->setPosition(CAN_ID, p);
        sleep(1);
        printf("--------------------\n");
        odrive->printInfo(CAN_ID);
    }

    odrive->setState(CAN_ID, MotorState::IDLE);

    return 0;
}