#include <stdio.h>
#include <unistd.h>

#include "starq/leg_controller.hpp"
#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"

#define LEG_ID 0x0

#define MOTOR_ID_0 0x0
#define MOTOR_ID_1 0x1

#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

using namespace starq;
using namespace starq::testing;
using namespace starq::dynamics;

int main(void)
{
    printf("Hello world!\n");

    DummyMotorController::Ptr dummy = std::make_shared<DummyMotorController>();
    printf("Created ODrive controller.\n");

    LegController::Ptr leg = std::make_shared<LegController>(dummy);
    printf("Created leg controller.\n");

    leg->setMotorIDs(LEG_ID, {MOTOR_ID_0, MOTOR_ID_1});
    printf("Set motor IDs.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(
        LEG_LINK_1_LENGTH_M,
        LEG_LINK_2_LENGTH_M);

    leg->setLegDynamics(LEG_ID, fivebar_dynamics);
    printf("Set leg dynamics.\n");

    printf("Starting leg movement in sine wave.\n");
    printf("\n");
    for (float t = 0.0f; t <= 2.0f * M_PI + 0.01; t += 0.1f)
    {
        const float center_x = 0.0f;
        const float center_y = -std::sqrt(2)*0.1;

        const float y_off = 0.025f * std::sin(t);

        VectorXf foot_position(2);
        foot_position << 0, center_y + y_off;

        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));

        leg->setFootPosition(LEG_ID, foot_position);

        VectorXf pos_estimate;
        leg->getFootPositionEstimate(LEG_ID, pos_estimate);

        printf("Foot position estimate: (%f, %f)\n", pos_estimate(0), pos_estimate(1));

        VectorXf joint_angles = leg->getCurrentJointAngles(LEG_ID);

        printf("Motor positions: (%f, %f)\n", joint_angles(0), joint_angles(1));

        const float force = -5.0;

        printf("Setting foot force to (%f, %f)\n", 0.0, force);

        leg->setFootForce(LEG_ID, Vector2f(0.0f, force));

        VectorXf motor_torques = leg->getCurrentJointTorques(LEG_ID);

        printf("Motor torques: (%f, %f)\n", motor_torques(0), motor_torques(1));

        printf("\n");

        usleep(5000);
    }

    return 0;
}