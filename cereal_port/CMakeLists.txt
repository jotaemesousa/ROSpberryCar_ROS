cmake_minimum_required(VERSION 2.8.3)
project(cereal_port)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cereal_port
#  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Boost_INCLUDE_DIRS}
  
)

add_library(cereal_port src/CerealPort.cpp)


#find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
#    include_directories(${Boost_INCLUDE_DIRS})

   # find_package(Threads REQUIRED)