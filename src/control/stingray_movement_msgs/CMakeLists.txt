cmake_minimum_required(VERSION 3.8)
project(stingray_movement_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "action/LinearMove.action"
        "action/Dive.action"
        "action/Rotate.action"
        "action/Align.action"
        "action/Tack.action"

        "msg/AlignValue.msg"
        )

ament_package()