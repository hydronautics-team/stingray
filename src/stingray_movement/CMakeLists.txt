cmake_minimum_required(VERSION 3.8)
project(stingray_movement)


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

add_library(${PROJECT_NAME} 
        include/stingray_movement/AbstractTwistActionServer.h
        include/stingray_movement/AbstractCenteringTwistActionServer.h
)
target_link_libraries(${PROJECT_NAME} rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces stingray_utils)

add_executable(twist_action_server
        include/stingray_movement/AbstractTwistActionServer.h
        include/stingray_movement/TwistActionServer.h
        src/TwistActionServer.cpp
        )
ament_target_dependencies(twist_action_server rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces stingray_utils)

add_executable(bbox_centering_twist_action_server
        include/stingray_movement/AbstractTwistActionServer.h
        include/stingray_movement/AbstractCenteringTwistActionServer.h
        include/stingray_movement/BboxCenteringTwistActionServer.h
        src/BboxCenteringTwistActionServer.cpp
        )
ament_target_dependencies(bbox_centering_twist_action_server rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces stingray_utils)

add_executable(bbox_search_twist_action_server
        include/stingray_movement/AbstractTwistActionServer.h
        include/stingray_movement/AbstractSearchTwistActionServer.h
        include/stingray_movement/BboxSearchTwistActionServer.h
        src/BboxSearchTwistActionServer.cpp
        )
ament_target_dependencies(bbox_search_twist_action_server rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces stingray_utils)

## Install headers
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

## Install executable
install(TARGETS
        twist_action_server
        bbox_centering_twist_action_server
        bbox_search_twist_action_server
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_package()