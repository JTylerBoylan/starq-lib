#include "starq/trajectories/trajectory_file_reader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace starq::trajectories;

TrajectoryFileReader::TrajectoryFileReader()
{
}

TrajectoryFileReader::~TrajectoryFileReader()
{
}

bool TrajectoryFileReader::load(const std::string &file_path)
{

    trajectory_.clear();

    std::ifstream file(file_path);

    if (!file.is_open())
    {
        std::cerr << "Could not open file " << file_path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        float time;
        uint8_t leg_id;
        uint32_t control_mode, input_mode;
        Eigen::Vector3f position, velocity, force;

        if (!(iss >>
              time >>
              leg_id >>
              control_mode >> input_mode >>
              position.x() >> position.y() >> position.z() >>
              velocity.x() >> velocity.y() >> velocity.z() >>
              force.x() >> force.y() >> force.z()))
        {
            std::cerr << "Error reading line " << line << std::endl;
            return false;
        }

        auto command = std::make_shared<starq::publishers::LegCommand>();
        command->leg_id = leg_id;
        command->control_mode = control_mode;
        command->input_mode = input_mode;
        command->target_position = position;
        command->target_velocity = velocity;
        command->target_force = force;
        
        trajectory_.push_back(command);
    }

    return true;
}