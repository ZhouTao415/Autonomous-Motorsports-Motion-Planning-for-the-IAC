cmake_minimum_required(VERSION 3.5)
project(behavior_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(route_planner REQUIRED)

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${route_planner_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
)

add_library(behavior_planner_lib
    src/behavior_planner/behavior_planner.cc
    src/behavior_planner/map_adapter.cc
)

target_link_libraries(behavior_planner_lib
    hkust_pl_common
    hkust_pl_rp
    hkust_pl_smm
    hkust_pl_vehicle_model
)

add_library(behavior_planner_ros
    src/behavior_planner/behavior_server_ros.cc
)

target_link_libraries(behavior_planner_ros
    behavior_planner_lib
    hkust_smm_ros
)

install(TARGETS
  behavior_planner_lib
  behavior_planner_ros
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
