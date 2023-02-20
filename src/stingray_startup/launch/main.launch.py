from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from stingray_utils.resources import load_config


def generate_launch_description():
    # load ros config

    return LaunchDescription([
        Node(
            package='stingray_gazebo_communication',
            executable='gazebo_bridge',
            name='gazebo_bridge'
        )
    ])
