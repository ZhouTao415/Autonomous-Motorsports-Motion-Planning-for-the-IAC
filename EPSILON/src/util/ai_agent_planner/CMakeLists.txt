cmake_minimum_required(VERSION 3.8)
project(ai_agent_planner)

# Enforce C++17 standard
set(CMAKE_CXX_STANDARD 17)
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
find_package(vehicle_msgs REQUIRED)


# Include directories for project and dependencies
include_directories(
    # ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
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
    common
    vehicle_model
    forward_simulator
    semantic_map_manager
    behavior_planner
    vehicle_msgs
)

# Link libraries
target_link_libraries(onlane_ai_agent
    dw
    ${catkin_INCLUDE_DIRS}
    ${rclcpp_LIBRARIES}
    ${common_LIBRARIES}
    ${semantic_map_manager_LIBRARIES}
    ${vehicle_model_LIBRARIES}
    ${forward_simulator_LIBRARIES}
    ${behavior_planner_LIBRARIES}
    ${vehicle_msgs_LIBRARIES}
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase 
    blas 
    ma27 
    gfortran 
)

# Install targets
install(TARGETS
  onlane_ai_agent
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
