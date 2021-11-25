cmake_minimum_required(VERSION 3.8)
project(stingray_movement)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(stingray_communication_msgs REQUIRED)
find_package(stingray_movement_msgs REQUIRED)

include_directories(
        include
)

add_executable(basic_movement
        include/common/AsyncTimer.h
        src/common/AsyncTimer.cpp
        include/common/AbstractMovementActionServer.h
        include/basic/LinearMovementServer.h
        src/basic/LinearMovementServer.cpp
        include/basic/DiveServer.h
        src/basic/DiveServer.cpp
        include/basic/RotateServer.h
        src/basic/RotateServer.cpp
        src/basic_movement.cpp
	)
ament_target_dependencies(basic_movement
        rclcpp
        std_msgs
        rclcpp_action
        stingray_communication_msgs
        stingray_movement_msgs
        )

add_executable(movement_patterns
        include/common/AsyncTimer.h
        src/common/AsyncTimer.cpp
        include/common/AbstractMovementActionServer.h
        include/patterns/AlignServer.h
        src/patterns/AlignServer.cpp
        include/patterns/AlignLowLevelServer.h
        src/patterns/AlignLowLevelServer.cpp
        include/patterns/TackServer.h
        src/patterns/TackServer.cpp
        src/movement_patterns.cpp)

ament_target_dependencies(basic_movement
        rclcpp
        std_msgs
        rclcpp_action
        stingray_communication_msgs
        stingray_movement_msgs)

install(TARGETS
        basic_movement
        movement_patterns
        DESTINATION lib/${PROJECT_NAME})
ament_package()
