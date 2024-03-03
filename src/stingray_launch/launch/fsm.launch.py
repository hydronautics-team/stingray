from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # load ros config
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('stingray_core_launch'), 'auv.launch.py')))),
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            # respawn=True,
            # respawn_delay=1,
        ),

        Node(
            package='stingray_movement',
            executable='twist_action_server',
            name='twist_action_server',
            # respawn=True,
            # respawn_delay=1,
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
        ),
        Node(
            package='stingray_missions',
            executable='qr_launcher',
            name='qr_launcher'
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            remappings=[
                ('/image_raw', '/stingray/topics/camera/front'),
            ]
        ),
    ])
