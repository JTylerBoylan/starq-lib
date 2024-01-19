#include <stdio.h>
#include <unistd.h>

#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"

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
    printf("Hello, world!\n");

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

    LegCommandPublisher::Ptr publisher = std::make_shared<LegCommandPublisher>(leg);
    printf("Created leg command publisher.\n");

    const float center_x = 0.0;
    const float center_y = -0.15;

    LegCommand cmd;
    cmd.leg_id = LEG_ID;
    cmd.control_mode = 0x3;
    cmd.target_position = Vector2f(center_x, center_y);
    cmd.target_velocity = VectorXf::Zero(2);
    cmd.target_force = VectorXf::Zero(2);
    cmd.delay_in_seconds = 0.0;

    publisher->push(cmd);
    printf("Pushed leg command.\n");

    usleep(10000);

    TrajectoryFileReader::Ptr reader = std::make_shared<TrajectoryFileReader>();
    printf("Created trajectory file reader.\n");

    reader->load("/app/trajectories/square.txt");
    printf("Loaded trajectory.\n");

    std::vector<LegCommand> trajectory = reader->getTrajectory();

    publisher->push(trajectory);
    printf("Pushed trajectory.\n");

    usleep(2E6);

    printf("Done.\n");

    return 0;
}