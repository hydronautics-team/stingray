from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # zbar camera
    zbar_camera_topic_arg = DeclareLaunchArgument(
        "zbar_camera_topic", default_value='/stingray/topics/camera'
    )
    # zbar
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )

    # load ros config
    return LaunchDescription([
        zbar_camera_topic_arg,
        zbar_topic_arg,

        # qr reader
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader',
            remappings=[
                ('/image', LaunchConfiguration("zbar_camera_topic")),
                ('/barcode', LaunchConfiguration("zbar_topic")),
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
