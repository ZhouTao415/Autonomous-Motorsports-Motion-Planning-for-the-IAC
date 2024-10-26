cmake_minimum_required(VERSION 3.5)
project(forward_simulator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3 -Wall)

# Set the CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find ament_cmake and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)

# Define include directories
include_directories(
    include
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
)

# Export libraries
ament_export_libraries(${vehicle_model_LIBRARIES})

ament_export_dependencies(
  rclcpp 
  visualization_msgs 
  sensor_msgs 
  common 
  vehicle_model
)


# Export include directories
ament_export_include_directories(
    include
    ${vehicle_model_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
