cmake_minimum_required(VERSION 3.8)
project(stingray_movement)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set (CMAKE_CXX_STANDARD  14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(actionlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(stingray_communication_msgs REQUIRED)
find_package(stingray_movement_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclidl_default_generators REQUIRED)

include_directories(include)

add_executable(basic_movement
  include/common/AsyncTimer.h
  src/common/AsyncTimer.cpp
  include/common/AbstractMovementActionServer.h
  include/basic/HorizontalMovementServer.h
  src/basic/HorizontalMovementServer.cpp
  include/basic/DiveServer.h
  src/basic/DiveServer.cpp
  src/basic_movement.cpp)

ament_target_dependencies(basic_movement 
                          rclcpp 
                          rclpy
                          actionlib
                          std_msgs 
                          actionlib_msgs
                          stingray_communication_msgs 
                          stingray_movement_msgs
                          geometry_msgs 
                          rclidl_default_generators)

add_dependencies(basic_movement ${stingray_communication_msgs_EXPORTED_TARGETS}
                 ${stingray_movement_msgs_EXPORTED_TARGETS})
