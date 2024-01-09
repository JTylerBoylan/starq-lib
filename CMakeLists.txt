cmake_minimum_required(VERSION 3.5)

project(starq_lib VERSION 1.0)

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(starq_lib SHARED)
target_sources(starq_lib PRIVATE
    src/can/socket.cpp
    src/controllers/leg_controller.cpp
    src/odrive/odrive_controller.cpp
    src/odrive/can_driver.cpp 
    src/odrive/can_listener.cpp
    src/dynamics/starq_fiverbar2d.cpp
)
target_include_directories(starq_lib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(starq_lib PUBLIC Eigen3::Eigen)
set_target_properties(starq_lib PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/usr/local/lib"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    starqLibConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS starq_lib EXPORT starqLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT starqLibTargets
    FILE starqLibTargets.cmake
    NAMESPACE starq_lib::
    DESTINATION lib/cmake/starq_lib
)

configure_file(cmake/starqLibConfig.cmake.in starqLibConfig.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/starqLibConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/starqLibConfigVersion.cmake"
        DESTINATION lib/cmake/starq_lib
)

install(DIRECTORY include/ DESTINATION include)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    include(CTest)
    enable_testing()

    add_executable(test_can_connection tests/test_can_connection.cpp)
    target_include_directories(test_can_connection PUBLIC include)
    target_link_libraries(test_can_connection PUBLIC stdc++ stdc++fs m starq_lib pthread)

    add_executable(test_odrive_control tests/test_odrive_control.cpp)
    target_include_directories(test_odrive_control PUBLIC include)
    target_link_libraries(test_odrive_control PUBLIC stdc++ stdc++fs m starq_lib pthread)

    add_executable(test_fivebar2d_position_control tests/test_fivebar2d_position_control.cpp)
    target_include_directories(test_fivebar2d_position_control PUBLIC include)
    target_link_libraries(test_fivebar2d_position_control PUBLIC stdc++ stdc++fs m starq_lib pthread)

endif()