cmake_minimum_required(VERSION 3.5)
project(eudm_planner)

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
find_package(semantic_map_manager REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(route_planner REQUIRED)
find_package(Protobuf REQUIRED)

# Protobuf generation
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/eudm_config.proto)

include_directories(
    include
    ${common_interfaces_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Define libraries
add_library(hkust_pl_eudm
    src/eudm_planner/eudm_planner.cc
    src/eudm_planner/map_adapter.cc
    src/eudm_planner/eudm_manager.cc
    src/eudm_planner/dcp_tree.cc
    ${PROTO_SRCS}
)
ament_target_dependencies(hkust_pl_eudm
    rclcpp visualization_msgs sensor_msgs common_interfaces vehicle_model semantic_map_manager forward_simulator route_planner Protobuf
)

add_library(hkust_eudm_ros
    src/eudm_planner/eudm_server_ros.cc
)
ament_target_dependencies(hkust_eudm_ros
    rclcpp visualization_msgs sensor_msgs common_interfaces vehicle_model semantic_map_manager forward_simulator route_planner Protobuf
)
target_link_libraries(hkust_eudm_ros
    hkust_pl_eudm
)

# Install targets and resources
install(TARGETS hkust_pl_eudm hkust_eudm_ros
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

ament_package()
