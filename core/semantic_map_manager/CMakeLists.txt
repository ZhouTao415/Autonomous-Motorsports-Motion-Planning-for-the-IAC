cmake_minimum_required(VERSION 3.5)
project(semantic_map_manager)

# Set C++ standard and build type
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Set additional CMake modules paths if necessary
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find necessary packages using ament and other package managers
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(motion_predictor REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(vehicle_model REQUIRED)

# Define include directories
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${vehicle_msgs_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Define the hkust_pl_smm library
add_library(hkust_pl_smm
    src/semantic_map_manager.cc
    src/data_renderer.cc
    src/config_loader.cc
    src/traffic_signal_manager.cc
)
ament_target_dependencies(hkust_pl_smm
    rclcpp
    vehicle_msgs
    vehicle_model
    common
    motion_predictor
    Eigen3
    OpenCV
)

# Define the hkust_smm_ros library
add_library(hkust_smm_ros
    src/ros_adapter.cc
    src/visualizer.cc
)
ament_target_dependencies(hkust_smm_ros
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    Eigen3
    OpenCV
    common
    motion_predictor
    vehicle_msgs
    vehicle_model
)
target_link_libraries(hkust_smm_ros
    hkust_pl_smm
)

# Exporting and installation setups
ament_export_libraries(hkust_pl_smm hkust_smm_ros)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
    OpenCV
    common
    motion_predictor
    vehicle_msgs
    vehicle_model
)

ament_export_include_directories(
    include
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${vehicle_msgs_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install the libraries
install(TARGETS hkust_pl_smm hkust_smm_ros
  EXPORT export_hkust_pl_smm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
