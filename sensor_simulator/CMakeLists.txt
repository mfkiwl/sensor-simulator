cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include(FetchContent)
FetchContent_Declare(NavFuse
        GIT_REPOSITORY git@github.com:ParkerBarrett959/NavFuse
        GIT_TAG main)
FetchContent_Declare(NavROS
        GIT_REPOSITORY git@github.com:ParkerBarrett959/NavROS
        GIT_TAG main)
FetchContent_MakeAvailable(NavROS)
FetchContent_MakeAvailable(NavFuse)
FetchContent_Declare(json
        GIT_REPOSITORY git@github.com:nlohmann/json
        GIT_TAG v3.11.2)
FetchContent_MakeAvailable(json)

# Add Header List
set(HEADER_LIST ${PROJECT_SOURCE_DIR}/include/sensor_simulator/imuSensor.hpp
                ${PROJECT_SOURCE_DIR}/include/sensor_simulator/looselyCoupledGpsSensor.hpp
                ${PROJECT_SOURCE_DIR}/include/sensor_simulator/util/trajectory.hpp
                ${PROJECT_SOURCE_DIR}/include/sensor_simulator/util/jsonUtilities.hpp)

# Include Directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/include/util)

# Add Executable
add_executable(SensorSim ${PROJECT_SOURCE_DIR}/src/main.cpp
                         ${PROJECT_SOURCE_DIR}/src/imuSensor.cpp
                         ${PROJECT_SOURCE_DIR}/src/looselyCoupledGpsSensor.cpp
                         ${PROJECT_SOURCE_DIR}/src/util/trajectory.cpp
                         ${PROJECT_SOURCE_DIR}/src/util/jsonUtilities.cpp ${HEADER_LIST})
ament_target_dependencies(SensorSim rclcpp)

target_link_libraries(
    SensorSim
    navigation
    nlohmann_json::nlohmann_json)

# Target Include Directories
target_include_directories(SensorSim PUBLIC ${PROJECT_SOURCE_DIR}/include/sensor_simulator
                                     PUBLIC ${PROJECT_SOURCE_DIR}/include/util/sensor_simulator}
                                     PUBLIC ${EIGEN_INSTALL_PATH})

# Unit Testing Build
enable_testing()
add_executable( test_sensor_sim ${PROJECT_SOURCE_DIR}/test/testMain.cpp
                                ${PROJECT_SOURCE_DIR}/test/trajectoryTest.cpp
                                ${PROJECT_SOURCE_DIR}/test/jsonUtilitiesTest.cpp
                                ${PROJECT_SOURCE_DIR}/src/util/trajectory.cpp
                                ${PROJECT_SOURCE_DIR}/src/util/jsonUtilities.cpp ${HEADER_LIST})

target_link_libraries(
  test_sensor_sim
  GTest::gtest_main
  navigation
  nlohmann_json::nlohmann_json
)

# Target Include Directories
target_include_directories(test_sensor_sim PUBLIC ${PROJECT_SOURCE_DIR}/include/sensor_simulator
                                           PUBLIC ${PROJECT_SOURCE_DIR}/include/util/sensor_simulator
                                           PUBLIC ${EIGEN_INSTALL_PATH})

include(GoogleTest)
gtest_discover_tests(test_sensor_sim)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
