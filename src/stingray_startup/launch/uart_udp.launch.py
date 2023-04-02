from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
	Node(
            package='stingray_communication',
            executable='uart_driver',
            name='uart_driver'
    ),
    Node(
        package='stingray_communication',
        executable='gui_bridge',
        name='gui_bridge'
    )
   ])