from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
	Node(
            package='stingray_communication',
            namespace='uartDriver',
            executable='uart_driver.cpp',
            name='uart_driver.cpp'
    ),
    Node(
        package='stingray_communication',
        executable='gui_bridge.cpp',
        name='gui_bridge.cpp'
    )
   ])