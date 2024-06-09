cmake_minimum_required(VERSION 3.5)
project(motion_predictor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall -fPIC)

# Set the CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)


# Define include directories
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

# Export libraries
ament_export_libraries(${forward_simulator_LIBRARIES})

# Declare ament package dependencies
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    vehicle_model
    forward_simulator
)

# Export include directories
ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
