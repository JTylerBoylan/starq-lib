#ifndef STARQ_SLAM__TERRAIN_MAP_HPP_
#define STARQ_SLAM__TERRAIN_MAP_HPP_

#include <memory>
#include "eigen3/Eigen/Dense"

namespace starq::slam
{
    using namespace Eigen;

    class TerrainMap
    {
    public:
        using Ptr = std::shared_ptr<TerrainMap>;

        /// @brief Create a terrain map.
        TerrainMap() {}

        /// @brief Destroy the terrain map.
        ~TerrainMap() {}

        /// @brief Get the distance to the nearest obstacle.
        /// @param position Position.
        /// @return Distance to the nearest obstacle.
        virtual double getDistanceToObstacle(const Vector3f &position) = 0;

    private:
    };

}

#endif