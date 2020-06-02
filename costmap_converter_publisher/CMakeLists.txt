cmake_minimum_required(VERSION 3.0.2)
project(costmap_converter_publisher)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  costmap_converter
  cv_bridge
  geometry_msgs
  pluginlib
  roscpp
  std_msgs
)

IF(NOT MSVC)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
    endif()
endif()

catkin_package(
        #INCLUDE_DIRS
        LIBRARIES costmap_converter
        CATKIN_DEPENDS
        geometry_msgs
        pluginlib
        roscpp
        std_msgs
        costmap_2d
        costmap_converter
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_executable(publish_costmap_conversion src/costmap_converter_node.cpp)
target_link_libraries(publish_costmap_conversion ${catkin_LIBRARIES} )

install(TARGETS publish_costmap_conversion
        RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION})

