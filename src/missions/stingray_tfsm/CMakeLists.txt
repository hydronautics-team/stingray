cmake_minimum_required(VERSION 2.8.3)
project(stingray_tfsm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  stingray_communication_msgs
  stingray_object_detection_msgs
)

catkin_package(
 INCLUDE_DIRS scripts
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        scripts/gate.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

