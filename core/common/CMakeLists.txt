cmake_minimum_required(VERSION 3.5)
project(common)

# Enforce C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Set additional CMake modules paths if necessary
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find necessary packages using ament and other package managers
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OOQP REQUIRED)

if(NOT OOQP_FOUND)
    message(FATAL_ERROR "OOQP not found.")
endif()

# Set directories for third-party and OOQP includes
set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty ${PROTOBUF_INCLUDE_DIR})
set(OOQP_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/OOQP/include)

# Include directories for project and dependencies
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/thirdparty
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${OOQP_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Link directories for third-party libraries
link_directories(${OOQP_LIBRARY_DIRS})

set(THIRD_PARTY_LIBRARIES
    dw
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OOQP_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

# Define the hkust_pl_ooqp_itf library
add_library(hkust_pl_ooqp_itf 
    src/common/solver/ooqp_interface.cc
    src/common/solver/qp_solver.cc
)
target_link_libraries(hkust_pl_ooqp_itf 
    ${THIRD_PARTY_LIBRARIES}
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran 
)

# Define the hkust_pl_common library with all common source files
set(COMMON_SRC
    src/common/basics/colormap.cc
    src/common/basics/semantics.cc
    src/common/basics/shapes.cc
    src/common/basics/tool_func.cc
    src/common/math/calculations.cc
    src/common/rss/rss_checker.cc
    src/common/idm/intelligent_driver_model.cc
    src/common/mobil/mobil_model.cc
    src/common/mobil/mobil_behavior_prediction.cc
    src/common/spline/lookup_table.cc
    src/common/spline/spline_generator.cc
    src/common/lane/lane.cc
    src/common/lane/lane_generator.cc
    src/common/primitive/frenet_primitive.cc
    src/common/state/state_transformer.cc
    src/common/state/free_state.cc
    src/common/circle_arc/circle_arc.cc
    src/common/circle_arc/circle_arc_branch.cc
)

add_library(hkust_pl_common 
    ${COMMON_SRC}
)
target_link_libraries(hkust_pl_common
    ${THIRD_PARTY_LIBRARIES}
    opencv_core
    opencv_highgui
    hkust_pl_ooqp_itf
    ${tf2_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_geometry_msgs_LIBRARIES}
)

# Exporting and installation setups
ament_export_libraries(hkust_pl_common hkust_pl_ooqp_itf)

ament_export_dependencies(
    rclcpp
    visualization_msgs
    sensor_msgs
    tf2_ros
    Eigen3
    Glog
    OpenCV
    Protobuf
    tf2
    tf2_ros
    tf2_geometry_msgs
)

ament_export_include_directories(
    include
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OOQP_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/thirdparty
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Define export sets for the targets
install(TARGETS hkust_pl_common hkust_pl_ooqp_itf
  EXPORT hkust_pl_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the export set
install(EXPORT hkust_pl_export
  FILE hkust_pl_export.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY materials/
  DESTINATION share/${PROJECT_NAME}/materials
)

ament_package()
