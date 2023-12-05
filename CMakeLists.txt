cmake_minimum_required(VERSION 3.5)

project(starq_sdk VERSION 1.0)

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(starq_sdk SHARED)
#target_sources(starq_sdk PRIVATE src/starq_sdk.cpp)
target_include_directories(starq_sdk PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(starq_sdk PUBLIC Eigen3::Eigen)
set_target_properties(starq_sdk PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/usr/local/lib"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    starqSdkConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS starq_sdk EXPORT starqSdkTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(EXPORT starqSdkTargets
    FILE starqSdkTargets.cmake
    NAMESPACE starq_sdk::
    DESTINATION lib/cmake/starq_sdk
)

configure_file(cmake/starqSdkConfig.cmake.in starqSdkConfig.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/starqSdkConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/starqSdkConfigVersion.cmake"
        DESTINATION lib/cmake/starq_sdk
)

install(DIRECTORY include/ DESTINATION include)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    enable_testing()

    # Add the executable
    add_executable(test_sdk tests/test_sdk.cpp)
    target_include_directories(test_sdk PUBLIC include)
    target_link_libraries(test_sdk PUBLIC stdc++ stdc++fs m starq_sdk)


endif()