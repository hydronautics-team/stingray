cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        rospy
        sensor_msgs
        std_msgs
        )

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        scripts/opencv_dnn_detector.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
