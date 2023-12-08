#ifndef STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__GAIT_CONTROLLER_HPP_

#include "starq/publishers/leg_command_publisher.hpp"
#include "starq/slam/localization.hpp"
#include "starq/slam/terrain_map.hpp"

namespace starq::controllers
{

    class GaitController
    {
    public:
        using Ptr = std::shared_ptr<GaitController>;

        /// @brief Create a gait controller.
        /// @param leg_command_publisher Leg command publisher.
        GaitController(const starq::publishers::LegCommandPublisher::Ptr leg_command_publisher,
                       const starq::slam::Localization::Ptr localization,
                       const starq::slam::TerrainMap::Ptr terrain_map)
            : leg_command_publisher_(leg_command_publisher),
              localization_(localization),
              terrain_map_(terrain_map) {}

        /// @brief Destroy the gait controller.
        ~GaitController() {}

        /// @brief Get the leg command publisher.
        /// @return Leg command publisher.
        starq::publishers::LegCommandPublisher::Ptr getLegCommandPublisher() { return leg_command_publisher_; }

        /// @brief Get the localization.
        /// @return Localization.
        starq::slam::Localization::Ptr getLocalization() { return localization_; }

        /// @brief Get the terrain map.
        /// @return Terrain map.
        starq::slam::TerrainMap::Ptr getTerrainMap() { return terrain_map_; }

        /// @brief Get the name of the gait.
        /// @return Name of the gait.
        virtual std::string getGaitName() = 0;

        /// @brief Move to a position.
        /// @param position Position to move to.
        virtual void moveTo(const Vector3f &position) = 0;

    private:
        starq::publishers::LegCommandPublisher::Ptr leg_command_publisher_;
        starq::slam::Localization::Ptr localization_;
        starq::slam::TerrainMap::Ptr terrain_map_;
    };

}

#endif