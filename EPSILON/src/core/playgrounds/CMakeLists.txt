cmake_minimum_required(VERSION 3.5)
project(playgrounds)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
  FILES_MATCHING PATTERN "*.json" PATTERN "*.xml"
)

ament_package()
