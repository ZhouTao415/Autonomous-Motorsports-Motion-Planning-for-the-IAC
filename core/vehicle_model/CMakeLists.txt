cmake_minimum_required(VERSION 3.5)
project(vehicle_model)

# Enforce C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Find ROS 2 and other package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(Boost REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)

# Define include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

# Define libraries
add_library(hkust_pl_vehicle_model 
    src/vehicle_model/vehicle_model.cc
    src/vehicle_model/ideal_steer_model.cc
    src/vehicle_model/idm_model.cc
    src/vehicle_model/ctx_idm_model.cc
    src/vehicle_model/controllers/pure_pursuit_controller.cc
    src/vehicle_model/controllers/pid_controller.cc
    src/vehicle_model/controllers/idm_velocity_controller.cc
    src/vehicle_model/controllers/ctx_idm_velocity_controller.cc
)


target_link_libraries(hkust_pl_vehicle_model
    hkust_pl_common
    ${Boost_LIBRARIES}
)

ament_export_libraries(hkust_pl_vehicle_model)
# Declare ament package dependencies
ament_export_dependencies(
    rclcpp 
    visualization_msgs 
    sensor_msgs 
    geometry_msgs 
    Eigen3 
    common
    Boost
)

# Export include directories
ament_export_include_directories(
    include 
    ${common_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

# Specify dependencies for the target
ament_target_dependencies(hkust_pl_vehicle_model
    rclcpp
    visualization_msgs 
    sensor_msgs 
    geometry_msgs 
    Eigen3 
    common
    Boost
)


install(DIRECTORY include/
  DESTINATION include
)

# Export libraries
install(TARGETS hkust_pl_vehicle_model
  EXPORT export_hkust_pl_vehicle_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
