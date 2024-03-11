cmake_minimum_required(VERSION 3.8)
project(stingray_utils)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

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

include_directories(include)

add_library(${PROJECT_NAME} 
        include/stingray_utils/AbstractActionServer.h
        include/stingray_utils/AsyncTimer.h
)
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
target_link_libraries(${PROJECT_NAME} rclcpp rclcpp_action rclcpp_components std_msgs std_srvs stingray_core_interfaces stingray_interfaces)



## Install headers
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)


## Install executable
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        # LIBRARY DESTINATION lib
        # ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
)




ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_package()