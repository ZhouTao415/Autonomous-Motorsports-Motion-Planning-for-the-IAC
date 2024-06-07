cmake_minimum_required(VERSION 3.5)
project(planning_integrated)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(eudm_planner REQUIRED)
find_package(ssc_planner REQUIRED)

ament_package()

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
)

add_executable(test_ssc_with_mpdm
    src/test_ssc_with_mpdm.cc
)
ament_target_dependencies(test_ssc_with_mpdm
   rclcpp
   message_filters
   visualization_msgs
   sensor_msgs
   common
   semantic_map_manager
   behavior_planner
   eudm_planner
   ssc_planner
)
target_link_libraries(test_ssc_with_mpdm
   ssc_server_ros
   behavior_planner_ros
)

add_executable(test_ssc_with_eudm
    src/test_ssc_with_eudm.cc
)
ament_target_dependencies(test_ssc_with_eudm
   rclcpp
   message_filters
   visualization_msgs
   sensor_msgs
   common
   semantic_map_manager
   behavior_planner
   eudm_planner
   ssc_planner
)
target_link_libraries(test_ssc_with_eudm
   ssc_server_ros
   hkust_eudm_ros
)

install(TARGETS test_ssc_with_mpdm test_ssc_with_eudm
  DESTINATION lib/${PROJECT_NAME})
