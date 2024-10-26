cmake_minimum_required(VERSION 3.5)
project(phy_simulator)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(vehicle_msgs REQUIRED)

# Include directories
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${semantics_msgs_INCLUDE_DIRS}
)

# Define library
add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc
    src/phy_simulator/phy_simulator.cc
)

target_link_libraries(phy_simulator_lib
    dw
    ${common_LIBRARIES}
    ${vehicle_model_LIBRARIES}
)

# Define executable
add_executable(phy_simulator_planning_node
    src/phy_simulator_planning_node.cc
    src/phy_simulator/visualizer.cc
    src/phy_simulator/ros_adapter.cc
)

target_link_libraries(phy_simulator_planning_node
    dw
    phy_simulator_lib
    ${common_LIBRARIES}
    ${vehicle_model_LIBRARIES}
    ${vehicle_msgs_LIBRARIES}
)

# Install the targets
install(TARGETS phy_simulator_lib phy_simulator_planning_node
  EXPORT export_phy_simulator
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Exporting and installation setups
ament_export_libraries(phy_simulator_lib)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    Eigen3
    common
    vehicle_model
    vehicle_msgs
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_package()
