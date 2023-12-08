#ifndef STARQ_TRAJECTORIES__TRAJECTORY_FILE_READER_HPP_
#define STARQ_TRAJECTORIES__TRAJECTORY_FILE_READER_HPP_

#include "starq/publishers/leg_command_publisher.hpp"

namespace starq::trajectories
{

    /// @brief Leg trajectory.
    struct LegTrajectory
    {
        using Ptr = std::shared_ptr<LegTrajectory>;

        size_t size;
        std::vector<starq::publishers::LegCommand::Ptr> trajectory;
    };

    class TrajectoryFileReader
    {

    public:
        using Ptr = std::shared_ptr<TrajectoryFileReader>;

        /// @brief Create a trajectory file reader.
        TrajectoryFileReader() {}

        /// @brief Destroy the trajectory file reader.
        ~TrajectoryFileReader() {}

        /// @brief Load a trajectory from a file.
        /// @param file_path Path to the file.
        /// @return Leg trajectory.
        LegTrajectory loadTrajectoryFromFile(const std::string &file_path);

    private:
    };

}

#endif