cmake_minimum_required(VERSION 3.0.2)
project(stingray_object_detection_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation
                                        std_msgs)

add_message_files(FILES Object.msg ObjectsArray.msg)

add_service_files(FILES SetEnableObjectDetection.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
