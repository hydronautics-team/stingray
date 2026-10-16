cmake_minimum_required(VERSION 3.8)
project(stingray_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTransition.srv"
  "srv/SetTwist.srv"
  "srv/SetStabilization.srv"
  "action/TwistAction.action"
  "action/BboxCenteringTwistAction.action"
  "action/BboxSearchTwistAction.action"
  "action/DeviceAction.action"
  "action/HydroacousticCenteringTwistAction.action"
  "action/NavigateToPose.action"
  "msg/Bbox.msg"
  "msg/BboxArray.msg"
  "msg/EnableTopic.msg"
  "msg/UVState.msg"
  "msg/ObjectsMap.msg"
  "msg/CordsDiff.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
