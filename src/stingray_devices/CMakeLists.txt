cmake_minimum_required(VERSION 3.8)
project(stingray_devices)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stingray_core_interfaces REQUIRED)
find_package(stingray_interfaces REQUIRED)
find_package(stingray_utils REQUIRED)

include_directories(
        include
)

add_executable(OpenCloseActionServer 
        src/OpenCloseActionServer.cpp 
        include/OpenCloseActionServer.h)
ament_target_dependencies(OpenCloseActionServer rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces stingray_utils)

install(TARGETS
        OpenCloseActionServer
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
