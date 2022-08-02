cmake_minimum_required(VERSION 2.8.3)
project(stingray_gazebo_communication)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             roslib
             std_msgs
             geometry_msgs
             sensor_msgs
             std_srvs
             nav_msgs
             stingray_communication_msgs
             stingray_utils)

catkin_package(
  DEPENDS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  nav_msgs
  serial
  stingray_communication_msgs
  stingray_utils)

include_directories(# include
                    ${catkin_INCLUDE_DIRS} ${stingray_utils_INCLUDE_DIRS})

add_executable(gazebo_bridge src/gazebo_bridge.cpp)

add_dependencies(gazebo_bridge
                 stingray_communication_msgs_generate_messages_cpp)

target_link_libraries(gazebo_bridge ${catkin_LIBRARIES})
