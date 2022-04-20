cmake_minimum_required(VERSION 2.8.3)
project(video_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)
include_directories(
)

install(PROGRAMS
        scripts/video_recorder.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
