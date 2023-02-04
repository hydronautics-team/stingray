cmake_minimum_required(VERSION 3.8)
project(stingray_communication_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(actionlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/HardwareInfo.msg"
  "srv/SetFloat64.srv"
  "srv/SetDeviceAction.srv"
  "srv/SetStabilization.srv"
  "srv/SetInt32.srv"
  "srv/SetHorizontalMove.srv"
  DEPENDENCIES  std_msgs
                geometry_msgs
                sensor_msgs
                actionlib_msgs
                )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
ament_export_dependencies(
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  actionlib_msgs
)
endif()

ament_package()
