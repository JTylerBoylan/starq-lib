#include "starq/trajectory_file_reader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace starq
{

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

            LegCommand command;
            if (!(iss >>
                  command.delay_in_seconds >>
                  command.leg_id >>
                  command.control_mode >> command.input_mode >>
                  command.target_position.x() >> command.target_position.y() >> command.target_position.z() >>
                  command.target_velocity.x() >> command.target_velocity.y() >> command.target_velocity.z() >>
                  command.target_force.x() >> command.target_force.y() >> command.target_force.z()))
            {
                std::cerr << "Error reading line " << line << std::endl;
                return false;
            }

            trajectory_.push_back(command);
        }

        return true;
    }

}