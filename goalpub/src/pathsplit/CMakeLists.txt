cmake_minimum_required(VERSION 2.8.3)
project(pathsplit)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg nav_msgs)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pathlist src/pathlist.cpp)
target_link_libraries(pathlist ${catkin_LIBRARIES})
add_dependencies(pathlist pathsplit_generate_messages_cpp)

add_executable(lidarod src/lidarod.cpp)
target_link_libraries(lidarod ${catkin_LIBRARIES})
add_dependencies(lidarod pathsplit_generate_messages_cpp)
