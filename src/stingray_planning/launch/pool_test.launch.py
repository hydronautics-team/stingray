import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('stingray_planning')
    planning_launch = os.path.join(package_share, 'launch', 'planning.launch.py')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_size = LaunchConfiguration('map_size')
    map_resolution = LaunchConfiguration('map_resolution')
    publish_map_odom_tf = LaunchConfiguration('publish_map_odom_tf')

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('publish_map_odom_tf', default_value='true'),
        DeclareLaunchArgument('map_size', default_value='20.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            output='screen',
            condition=IfCondition(publish_map_odom_tf),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', map_frame,
                '--child-frame-id', odom_frame,
            ],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(planning_launch)),
        Node(
            package='stingray_planning',
            executable='empty_map_publisher',
            name='pool_empty_map',
            output='screen',
            parameters=[{
                'frame_id': map_frame,
                'size_m': ParameterValue(map_size, value_type=float),
                'resolution': ParameterValue(map_resolution, value_type=float),
            }],
        ),
    ])
