from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('stingray_planning')
    params_file = os.path.join(pkg_share, 'config', 'planning_params.yaml')

    path_planner = Node(
        package='stingray_planning',
        executable='path_planning_node',
        name='path_planning',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        path_planner,
    ])
