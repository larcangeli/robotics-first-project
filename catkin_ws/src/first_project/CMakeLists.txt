cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
  message_filters
)

add_message_files(
  FILES
  sector_times.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_filters message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometer src/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

add_executable(sector_times src/sector_times.cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})

add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sector_times ${catkin_EXPORTED_TARGETS} first_project_generate_messages_cpp)
