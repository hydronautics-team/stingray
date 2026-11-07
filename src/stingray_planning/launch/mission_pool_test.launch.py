import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('stingray_planning')
    pool_test_launch = os.path.join(
        package_share, 'launch', 'pool_test.launch.py')

    map_frame = LaunchConfiguration('map_frame')
    map_size = LaunchConfiguration('map_size')
    map_resolution = LaunchConfiguration('map_resolution')

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('map_size', default_value='20.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pool_test_launch),
            launch_arguments={
                'map_frame': map_frame,
                'map_size': map_size,
                'map_resolution': map_resolution,
            }.items(),
        ),
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
