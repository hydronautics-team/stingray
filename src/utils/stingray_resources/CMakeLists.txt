cmake_minimum_required(VERSION 3.0.2)
project(stingray_resources)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(INCLUDE_DIRS scripts)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
