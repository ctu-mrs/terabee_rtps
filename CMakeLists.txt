cmake_minimum_required(VERSION 2.8.3)
project(rtps_republisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  mrs_lib
  positioning_systems_ros
)

catkin_package()

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/rtps_republisher.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
