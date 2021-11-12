cmake_minimum_required(VERSION 3.8)
project(stingray_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_communication_msgs REQUIRED)


include_directories(
        include
)

#add_executable(gazebo_bridge
#        src/gazebo_bridge.cpp
#        include/topics_and_services.h
#        src/messages/messages.cpp
#        include/messages/messages.h)
#ament_target_dependencies(gazebo_bridge rclcpp std_msgs geometry_msgs sensor_msgs std_srvs nav_msgs serial stingray_communication_msgs)

add_executable(uart_bridge
        src/uart_bridge.cpp
        include/uart_bridge.h
        include/topics_and_services.h
        src/messages/messages.cpp
        include/messages/messages.h)
ament_target_dependencies(uart_bridge rclcpp std_msgs geometry_msgs sensor_msgs std_srvs nav_msgs serial stingray_communication_msgs)

add_executable(hardware_bridge
        src/hardware_bridge.cpp
        include/hardware_bridge.h
        include/topics_and_services.h
        src/messages/messages.cpp
        include/messages/messages.h)
ament_target_dependencies(hardware_bridge rclcpp std_msgs geometry_msgs sensor_msgs std_srvs nav_msgs serial stingray_communication_msgs)

install(TARGETS
        hardware_bridge
        uart_bridge
#        gazebo_bridge
        DESTINATION lib/${PROJECT_NAME})

ament_package()
