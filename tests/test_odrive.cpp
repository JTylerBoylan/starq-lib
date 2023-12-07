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

    const float position_increment = 0.2;
    for (float p = 0.0f; p < 1.0f; p += position_increment)
    {
        printf("Sending command...\n");
        odrive->setPosition(CAN_ID, p);
        printf("Set position to %.5f\n", p);

        const float pos_estimate = odrive->getPositionEstimate(CAN_ID);
        printf("Current position estimate: %.5f\n", pos_estimate);
        sleep(1);
    }

    odrive->setAxisState(CAN_ID, AxisState::IDLE);

    return 0;
}