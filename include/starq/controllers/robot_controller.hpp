#ifndef STARQ_CONTROLLERS__ROBOT_CONTROLLER_HPP_
#define STARQ_CONTROLLERS__ROBOT_CONTROLLER_HPP_

#include <memory>

#include "starq/controllers/motor_controller.hpp"
#include "starq/controllers/leg_controller.hpp"
#include "starq/publishers/leg_command_publisher.hpp"
#include "starq/slam/localization.hpp"
#include "starq/slam/terrain_map.hpp"

namespace starq::controllers
{

    class RobotController
    {
    public:
        using Ptr = std::shared_ptr<RobotController>;

        /// @brief Create a robot controller.
        RobotController(MotorController::Ptr motor_controller,
                        slam::Localization::Ptr localization,
                        slam::TerrainMap::Ptr terrain_map)
            : motor_controller_(motor_controller),
              leg_controller_(std::make_shared<LegController>(motor_controller)),
              leg_command_publisher_(std::make_shared<publishers::LegCommandPublisher>(leg_controller_)),
              localization_(localization),
              terrain_map_(terrain_map)
        {
        }

        /// @brief Destroy the robot controller.
        ~RobotController() {}

        /// @brief Get the motor controller.
        /// @return Motor controller.
        MotorController::Ptr getMotorController() { return motor_controller_; }

        /// @brief Get the leg controller.
        /// @return Leg controller.
        LegController::Ptr getLegController() { return leg_controller_; }

        /// @brief Get the leg command publisher.
        /// @return Leg command publisher.
        publishers::LegCommandPublisher::Ptr getLegCommandPublisher() { return leg_command_publisher_; }

        /// @brief Get the localization.
        /// @return Localization.
        slam::Localization::Ptr getLocalization() { return localization_; }

        /// @brief Get the terrain map.
        /// @return Terrain map.
        slam::TerrainMap::Ptr getTerrainMap() { return terrain_map_; }

    private:
        MotorController::Ptr motor_controller_;
        LegController::Ptr leg_controller_;
        publishers::LegCommandPublisher::Ptr leg_command_publisher_;
        slam::Localization::Ptr localization_;
        slam::TerrainMap::Ptr terrain_map_;
    };

}

#endif