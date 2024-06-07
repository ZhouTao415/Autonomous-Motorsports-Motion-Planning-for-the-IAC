cmake_minimum_required(VERSION 3.5)
project(ssc_planner)

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
find_package(OpenMP REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(Protobuf REQUIRED)

# Protobuf generation
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/ssc_config.proto)

include_directories(
    include
    ${common_interfaces_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Define libraries
add_library(hkust_pl_ssc
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
    ${PROTO_SRCS}
)
ament_target_dependencies(hkust_pl_ssc
    rclcpp
    visualization_msgs
    sensor_msgs
    common_interfaces
    semantic_map_manager
    Protobuf
)
target_link_libraries(hkust_pl_ssc PRIVATE
    ${OpenMP_CXX_FLAGS}
)

add_library(ssc_server_ros
    src/ssc_planner/ssc_server_ros.cc
    src/ssc_planner/ssc_visualizer.cc
)
ament_target_dependencies(ssc_server_ros
    rclcpp
    visualization_msgs
    sensor_msgs
    common_interfaces
    semantic_map_manager
    Protobuf
)
target_link_libraries(ssc_server_ros PRIVATE
    hkust_pl_ssc
    ${OpenMP_CXX_FLAGS}
)

# Install targets and resources
install(TARGETS hkust_pl_ssc ssc_server_ros
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
