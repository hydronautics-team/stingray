cmake_minimum_required(VERSION 2.8.3)
project(stingray_communication_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        actionlib_msgs
        )

add_service_files(
        FILES
        SetFloat64.srv
        SetDeviceAction.srv
        SetStabilization.srv
        SetInt32.srv
        SetLagAndMarch.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs message_runtime actionlib_msgs
        #  INCLUDE_DIRS include
        #  LIBRARIES stingray_common
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
