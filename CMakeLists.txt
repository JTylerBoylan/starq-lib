cmake_minimum_required(VERSION 3.5)

project(starq VERSION 1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(starq SHARED)
target_sources(starq PRIVATE
    src/leg_controller.cpp
    src/can/socket.cpp
    src/odrive/odrive_controller.cpp
    src/odrive/can_driver.cpp 
    src/odrive/can_listener.cpp
    src/dynamics/starq_fiverbar2d.cpp
)
target_include_directories(starq PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(starq PUBLIC Eigen3::Eigen)
set_target_properties(starq PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/usr/local/lib"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    starqConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS starq EXPORT starqTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT starqTargets
    FILE starqTargets.cmake
    NAMESPACE starq::
    DESTINATION lib/cmake/starq
)

configure_file(cmake/starqConfig.cmake.in starqConfig.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/starqConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/starqConfigVersion.cmake"
        DESTINATION lib/cmake/starq
)

install(DIRECTORY include/ DESTINATION include)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    include(CTest)
    enable_testing()

    add_executable(test_can_connection tests/test_can_connection.cpp)
    target_include_directories(test_can_connection PUBLIC include)
    target_link_libraries(test_can_connection PUBLIC stdc++ stdc++fs m starq pthread)

    add_executable(test_odrive_control tests/test_odrive_control.cpp)
    target_include_directories(test_odrive_control PUBLIC include)
    target_link_libraries(test_odrive_control PUBLIC stdc++ stdc++fs m starq pthread)

    add_executable(test_fivebar2d_position_control tests/test_fivebar2d_position_control.cpp)
    target_include_directories(test_fivebar2d_position_control PUBLIC include)
    target_link_libraries(test_fivebar2d_position_control PUBLIC stdc++ stdc++fs m starq pthread)

    add_executable(test_fivebar2d_force_control tests/test_fivebar2d_force_control.cpp)
    target_include_directories(test_fivebar2d_force_control PUBLIC include)
    target_link_libraries(test_fivebar2d_force_control PUBLIC stdc++ stdc++fs m starq pthread)

    add_executable(test_fivebar2d_dummy tests/test_fivebar2d_dummy.cpp)
    target_include_directories(test_fivebar2d_dummy PUBLIC include)
    target_link_libraries(test_fivebar2d_dummy PUBLIC stdc++ stdc++fs m starq pthread)

endif()