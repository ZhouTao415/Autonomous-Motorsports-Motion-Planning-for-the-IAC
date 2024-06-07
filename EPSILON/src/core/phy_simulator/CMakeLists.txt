cmake_minimum_required(VERSION 3.5)
project(phy_simulator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(vehicle_msgs REQUIRED)

# Define include directories
include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    
)


# message("Include directories: ${vehicle_msgs_INCLUDE_DIRS}")


# Declare ament package dependencies
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    Eigen3
    common
    vehicle_model
    vehicle_msgs
)

# Define libraries
add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc
    src/phy_simulator/phy_simulator.cc
)
target_link_libraries(phy_simulator_lib
    hkust_pl_vehicle_model
    hkust_pl_common
)

ament_target_dependencies(phy_simulator_lib
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    Eigen3
    common
    vehicle_model
    vehicle_msgs
)

# Define executables
add_executable(phy_simulator_planning_node
    src/phy_simulator_planning_node.cc
    src/phy_simulator/visualizer.cc
    src/phy_simulator/ros_adapter.cc
)
target_link_libraries(phy_simulator_planning_node
    phy_simulator_lib
    ${rclcpp_LIBRARIES}
)

ament_target_dependencies(phy_simulator_planning_node
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    Eigen3
    common
    vehicle_model
    vehicle_msgs
)

# Export include directories and libraries
ament_export_include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

ament_export_libraries(phy_simulator_lib)

# Install libraries
install(TARGETS phy_simulator_lib
  ARCHIVE DESTINATION lib  # Static libraries
  LIBRARY DESTINATION lib  # Shared libraries
  RUNTIME DESTINATION bin  # Executables/DLLs
)


# Install executables
install(TARGETS phy_simulator_planning_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # Executables
  LIBRARY DESTINATION lib/${PROJECT_NAME}  # Shared libraries if any
  ARCHIVE DESTINATION lib/${PROJECT_NAME}  # Static libraries if any
)

#install the hearder files so that other packages can include.
install(DIRECTORY inc/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
