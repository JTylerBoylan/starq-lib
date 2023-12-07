#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq::can;
using namespace starq::odrive;

#define CAN_ID 0

int main(void)
{
    printf("Hello world!\n");

    CANSocket::Ptr socket = std::make_shared<CANSocket>("can0");

    ODriveController::Ptr odrive = std::make_shared<ODriveController>(socket);

    odrive->setAxisState(CAN_ID, AxisState::CLOSED_LOOP_CONTROL);
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

    odrive->setAxisState(CAN_ID, AxisState::IDLE);

    return 0;
}