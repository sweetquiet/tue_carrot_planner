cmake_minimum_required(VERSION 2.8.3)
project(tue_carrot_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tue_move_base_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tue_carrot_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(HEADER_FILES include/tue_carrot_planner/carrot_planner.h)

add_library(tue_carrot_planner src/carrot_planner.cpp ${HEADER_FILES})
target_link_libraries(tue_carrot_planner ${catkin_LIBRARIES})
