import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # load ros config
    return LaunchDescription([
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            # respawn=True,
            # respawn_delay=1,
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
        ),
    ])
