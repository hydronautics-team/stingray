import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('stingray_planning')
    planning_launch = os.path.join(package_share, 'launch', 'planning.launch.py')
    map_size = LaunchConfiguration('map_size')
    map_resolution = LaunchConfiguration('map_resolution')

    return LaunchDescription([
        DeclareLaunchArgument('map_size', default_value='20.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(planning_launch)),
        Node(
            package='stingray_planning',
            executable='empty_map_publisher',
            name='pool_empty_map',
            output='screen',
            parameters=[{
                'frame_id': 'odom',
                'size_m': ParameterValue(map_size, value_type=float),
                'resolution': ParameterValue(map_resolution, value_type=float),
            }],
        ),
    ])
