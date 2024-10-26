cmake_minimum_required(VERSION 3.5)
project(vehicle_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common REQUIRED)

# Include directories
include_directories(
  include
  ${common_INCLUDE_DIRS}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/ControlSignal.msg"
  "msg/Lane.msg"
  "msg/LaneNet.msg"
  "msg/VehicleParam.msg"
  "msg/Vehicle.msg"
  "msg/VehicleSet.msg"
  "msg/OccupancyGridFloat.msg"
  "msg/OccupancyGridUInt8.msg"
  "msg/Circle.msg"
  "msg/CircleObstacle.msg"
  "msg/PolygonObstacle.msg"
  "msg/ObstacleSet.msg"
  "msg/FreeState.msg"
  "msg/ArenaInfo.msg"
  "msg/ArenaInfoDynamic.msg"
  "msg/ArenaInfoStatic.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)

ament_export_include_directories(
    include
    ${common_INCLUDE_DIRS}
)
ament_export_dependencies(rosidl_default_runtime)


# Install the package share items
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
