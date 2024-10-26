cmake_minimum_required(VERSION 3.5)
project(eudm_planner)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(Protobuf REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(route_planner REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(Glog REQUIRED)  # Add this line

# Protobuf generation
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS 
    proto/eudm_config.proto 
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


# Include directories
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}  # Add this line
)

# Define libraries
add_library(hkust_pl_eudm
    src/eudm_planner/eudm_planner.cc
    src/eudm_planner/map_adapter.cc
    src/eudm_planner/eudm_manager.cc
    src/eudm_planner/dcp_tree.cc
    ${PROTO_SRCS}
)
target_link_libraries(hkust_pl_eudm
    hkust_pl_smm
    ${Protobuf_LIBRARIES}
    glog::glog  # Add this line
)

add_library(hkust_eudm_ros
    src/eudm_planner/eudm_server_ros.cc
)
target_link_libraries(hkust_eudm_ros
    hkust_pl_eudm
    hkust_smm_ros
    ${Protobuf_LIBRARIES}
    glog::glog  # Add this line
)

ament_export_libraries(hkust_pl_eudm hkust_eudm_ros)

# Exporting and installation setups
ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    Protobuf
    common
    vehicle_model
    semantic_map_manager
    forward_simulator
    route_planner
    vehicle_msgs
    glog  # Add this line
)

ament_export_include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${vehicle_msgs_INCLUDE_DIRS}
)

# Install the targets
install(TARGETS hkust_pl_eudm hkust_eudm_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
