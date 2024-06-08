cmake_minimum_required(VERSION 3.5)
project(route_planner)

# Enforce C++14 standard
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
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)

# Include directories
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

# Define library
add_library(hkust_pl_rp 
    src/route_planner/route_planner.cc
)

target_link_libraries(hkust_pl_rp
    hkust_pl_common
    hkust_pl_smm
)

# Exporting and installation setups
ament_export_libraries(hkust_pl_rp)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    common
    semantic_map_manager
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

# Install the targets
install(TARGETS hkust_pl_rp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
