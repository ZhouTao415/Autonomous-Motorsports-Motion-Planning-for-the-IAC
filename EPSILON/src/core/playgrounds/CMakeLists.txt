cmake_minimum_required(VERSION 3.5)
project(playgrounds)

# Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  highway_lite 
  highway_v1.0 
  ring_small_v1.0 
  ring_tiny_v1.0 
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  FILES_MATCHING PATTERN "*.json" PATTERN "*.xml"
)

install(DIRECTORY highway_lite highway_v1.0 ring_small_v1.0 ring_tiny_v1.0 DESTINATION share/${PROJECT_NAME})

ament_package()
