cmake_minimum_required(VERSION 3.5)
project(ssc_planner)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Set CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(Protobuf REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(Glog REQUIRED)  # Add this line

# Protobuf generation
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/ssc_config.proto
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# Include directories
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}  # Add this line
)

# Define libraries
add_library(hkust_pl_ssc
    src/ssc_planner/ssc_planner.cc
    src/ssc_planner/ssc_map.cc
    src/ssc_planner/map_adapter.cc
    ${PROTO_SRCS}
)
target_link_libraries(hkust_pl_ssc
    hkust_pl_common
    hkust_pl_smm
    OpenMP::OpenMP_CXX
    ${Protobuf_LIBRARIES}
    glog::glog  # Correctly link glog
)

add_library(ssc_server_ros
    src/ssc_planner/ssc_server_ros.cc
    src/ssc_planner/ssc_visualizer.cc
)
target_link_libraries(ssc_server_ros
    hkust_pl_ssc
    hkust_smm_ros
    OpenMP::OpenMP_CXX
)

# Exporting and installation setups
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    semantic_map_manager
    Protobuf
    OpenMP
    vehicle_msgs
    glog  # Add glog to the dependencies
)

ament_export_include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
)

ament_export_libraries(hkust_pl_ssc ssc_server_ros)

# Install the targets
install(TARGETS hkust_pl_ssc ssc_server_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
