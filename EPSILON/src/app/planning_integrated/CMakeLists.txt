cmake_minimum_required(VERSION 3.5)
project(planning_integrated)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(eudm_planner REQUIRED)
find_package(ssc_planner REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



# Include directories
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)


# Define executables and link against libraries
add_executable(test_ssc_with_mpdm src/test_ssc_with_mpdm.cc)
ament_target_dependencies(test_ssc_with_mpdm
  rclcpp
  visualization_msgs
  sensor_msgs
  message_filters
  common
  semantic_map_manager
  behavior_planner
  ssc_planner
  Protobuf
  glog
)

target_link_libraries(test_ssc_with_mpdm
  dw
  ${behavior_planner_LIBRARIES}
  ${ssc_planner_LIBRARIES}
  ${Protobuf_LIBRARIES}
  glog::glog  # Correctly link glog
  ${OpenMP_CXX_LIBRARIES} 
  ooqpgensparse
  ooqpsparse 
  ooqpgondzio 
  ooqpbase blas ma27 gfortran 
)

add_executable(test_ssc_with_eudm src/test_ssc_with_eudm.cc)
ament_target_dependencies(test_ssc_with_eudm
  rclcpp
  visualization_msgs
  sensor_msgs
  message_filters
  common
  semantic_map_manager
  eudm_planner
  ssc_planner
  Protobuf
  glog
)

target_link_libraries(test_ssc_with_eudm
  dw
  ${ssc_planner_LIBRARIES}
  ${eudm_planner_LIBRARIES}
  ${Protobuf_LIBRARIES}
  glog::glog  # Correctly link glog
  ooqpgensparse
  ooqpsparse 
  ooqpgondzio 
  ooqpbase blas ma27 gfortran 
)

# Exporting and installation setups
ament_export_dependencies(test_ssc_with_eudm
    rclcpp
    visualization_msgs
    sensor_msgs
    message_filters
    common
    semantic_map_manager
    behavior_planner
    eudm_planner
    ssc_planner
    glog
    Protobuf
)

ament_export_include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

# Install the targets
# install(TARGETS test_ssc_with_mpdm test_ssc_with_eudm
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(TARGETS 
  test_ssc_with_mpdm 
  test_ssc_with_eudm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
