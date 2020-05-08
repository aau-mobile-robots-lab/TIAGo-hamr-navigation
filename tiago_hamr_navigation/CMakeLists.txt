cmake_minimum_required(VERSION 2.8.3)
project(tiago_hamr_navigation)

find_package(catkin REQUIRED COMPONENTS
  amcl
  dwa_local_planner
  move_base
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})


