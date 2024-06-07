cmake_minimum_required(VERSION 3.5)
project(semantic_map_manager)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

# Find ROS 2 and other package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(motion_predictor REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



include_directories(
    inc
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${motion_predictor_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}

    /home/tao/Desktop/AD/ad_pomdp/src/core/forward_simulator/inc
    /home/tao/Desktop/AD/ad_pomdp/src/core/vehicle_msgs/inc

    

    
)
# message("Include directories: ${CMAKE_SOURCE_DIR}/src/core/forward_simulator/inc")
message(STATUS "Project root directory: ${CMAKE_SOURCE_DIR}")

# message("Include directories: ${motion_predictor_INCLUDE_DIRS}")

    


ament_export_include_directories(inc)

# Declare ament package dependencies
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    OpenCV
    Eigen3
    common
    vehicle_msgs
    vehicle_model
    motion_predictor
    forward_simulator
    tf2_geometry_msgs

)
# Define libraries
add_library(hkust_pl_smm
    src/semantic_map_manager.cc
    src/data_renderer.cc
    src/config_loader.cc
    src/traffic_signal_manager.cc
)
ament_target_dependencies(hkust_pl_smm
    rclcpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    OpenCV
    Eigen3
    common
    vehicle_msgs
    vehicle_model
    motion_predictor
    forward_simulator
    tf2_geometry_msgs

)

add_library(hkust_smm_ros
    src/ros_adapter.cc
    src/visualizer.cc
)
target_link_libraries(hkust_smm_ros
    hkust_pl_smm
    ${OpenCV_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${motion_predictor_LIBRARIES}
)

# Install libraries, executables, and headers
install(TARGETS hkust_pl_smm hkust_smm_ros
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION inc
)


#install the hearder files so that other packages can include.
# install(DIRECTORY inc/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )


install(DIRECTORY inc/${PROJECT_NAME}/
  DESTINATION inc/${PROJECT_NAME}  # Corrected to a single, valid destination
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()