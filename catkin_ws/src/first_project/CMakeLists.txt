cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

