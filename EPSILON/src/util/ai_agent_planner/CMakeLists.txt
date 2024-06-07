cmake_minimum_required(VERSION 3.5)
project(ai_agent_planner)

# Enforce C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)

# Include directories
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
)

# Define executable
add_executable(onlane_ai_agent
    src/onlane_ai_agent.cc
)
target_link_libraries(onlane_ai_agent
    ${rclcpp_LIBRARIES}
    hkust_smm_ros
    behavior_planner_ros
    hkust_pl_vehicle_model
)

ament_export_libraries(onlane_ai_agent)

# Exporting and installation setups
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    vehicle_model
    forward_simulator
    semantic_map_manager
    behavior_planner
)

ament_export_include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
)

# Install the targets
install(TARGETS onlane_ai_agent
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
