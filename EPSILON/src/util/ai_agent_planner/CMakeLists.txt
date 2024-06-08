cmake_minimum_required(VERSION 3.5)
project(ai_agent_planner)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Set additional CMake modules paths if necessary
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find necessary packages using ament and other package managers
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)

# Include directories for project and dependencies
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
)

add_executable(onlane_ai_agent
    src/onlane_ai_agent.cc
)
target_link_libraries(onlane_ai_agent
    ${catkin_LIBRARIES}
    hkust_smm_ros
    behavior_planner_ros
    hkust_pl_vehicle_model
)

ament_package()
