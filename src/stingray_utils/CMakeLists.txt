cmake_minimum_required(VERSION 3.8)
project(stingray_utils)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stingray_core_interfaces REQUIRED)
find_package(stingray_interfaces REQUIRED)

set(utils_SRCS
    include/stingray_utils/AbstractActionServer.h
    include/stingray_utils/AsyncTimer.h
)
add_library(${PROJECT_NAME} ${utils_SRCS})
target_link_libraries(${PROJECT_NAME} rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces)

include_directories(include)

## Install executable
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

## Install headers
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_package()