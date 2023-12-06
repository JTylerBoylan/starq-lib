#include <stdio.h>

#include "starq/can/socket.hpp"

int main(void)
{
    printf("Hello world!\n");

    starq::can::CANSocket socket("can0");

    return 0;
}