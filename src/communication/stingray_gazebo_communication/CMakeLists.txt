cmake_minimum_required(VERSION 3.8)
project(stingray_gazebo_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_communication_msgs REQUIRED)
find_package(stingray_utils REQUIRED)
find_package(gazebo_msgs REQUIRED)


include_directories(
        include
        ${stingray_utils_INCLUDE_DIRS}
)

add_executable(gazebo_bridge src/gazebo_bridge.cpp)
ament_target_dependencies(gazebo_bridge rclcpp std_msgs std_srvs serial stingray_communication_msgs stingray_utils gazebo_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS
        gazebo_bridge
        DESTINATION lib/${PROJECT_NAME})

ament_package()