cmake_minimum_required(VERSION 3.5)
project(route_planner)

# Use C++ 14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(semantic_map_manager REQUIRED)

# Declare include directories
include_directories(
    include
    ${common_interfaces_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

# Create a library
add_library(hkust_pl_rp
    src/route_planner/route_planner.cc
)

# Link libraries
ament_target_dependencies(hkust_pl_rp
    rclcpp
    visualization_msgs
    sensor_msgs
    common_interfaces
    semantic_map_manager
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(hkust_pl_rp)

# Install the library
install(TARGETS hkust_pl_rp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

ament_package()
