cmake_minimum_required(VERSION 3.5)
project(planning_integrated)

# Set build options
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -g)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(eudm_planner REQUIRED)
find_package(ssc_planner REQUIRED)

# Include directories
include_directories(
   ${rclcpp_INCLUDE_DIRS}
   ${common_INCLUDE_DIRS}
   ${semantic_map_manager_INCLUDE_DIRS}
   ${behavior_planner_INCLUDE_DIRS}
   ${eudm_planner_INCLUDE_DIRS}
   ${ssc_planner_INCLUDE_DIRS}
)

# Define and link the executables
add_executable(test_ssc_with_mpdm
    src/test_ssc_with_mpdm.cc
)
target_link_libraries(test_ssc_with_mpdm
   ${catkin_LIBRARIES}
   ssc_server_ros
   behavior_planner_ros
)

add_executable(test_ssc_with_eudm
    src/test_ssc_with_eudm.cc
)
target_link_libraries(test_ssc_with_eudm
   ${catkin_LIBRARIES}
   ssc_server_ros
   hkust_eudm_ros
)

ament_package()
