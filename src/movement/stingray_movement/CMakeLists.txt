cmake_minimum_required(VERSION 3.8)
project(stingray_movement)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stingray_communication_msgs REQUIRED)
find_package(stingray_movement_msgs REQUIRED)

include_directories(
  include
)

add_executable(basic_movement
  include/common/AsyncTimer.h
  src/common/AsyncTimer.cpp
  include/common/AbstractMovementActionServer.h
  include/basic/HorizontalMovementServer.h
  src/basic/HorizontalMovementServer.cpp
  include/basic/DiveServer.h
  src/basic/DiveServer.cpp
  src/basic_movement.cpp)
ament_target_dependencies(basic_movement rclcpp std_msgs stingray_communication_msgs stingray_movement_msgs)

install(TARGETS
        basic_movement
        DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()




