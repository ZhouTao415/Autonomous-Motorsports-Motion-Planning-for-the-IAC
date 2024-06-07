cmake_minimum_required(VERSION 3.5)
project(ai_agent_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Find ROS 2 and other package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)

include_directories(
    include
    ${common_interfaces_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
)

add_executable(onlane_ai_agent
    src/onlane_ai_agent.cc
)
ament_target_dependencies(onlane_ai_agent
    rclcpp
    visualization_msgs
    sensor_msgs
    common_interfaces
    vehicle_model
    forward_simulator
    semantic_map_manager
    behavior_planner
)

# Link libraries
target_link_libraries(onlane_ai_agent
    hkust_smm_ros
    behavior_planner_ros
    hkust_pl_vehicle_model
)

# Install targets and resources
install(TARGETS onlane_ai_agent
    DESTINATION lib/${PROJECT_NAME}
)

# Optionally install header files
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION include/${PROJECT_ NAME}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)

ament_package()
