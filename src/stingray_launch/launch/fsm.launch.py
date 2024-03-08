import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

transition_srv = '/stingray/services/transition'
zbar_topic = '/stingray/topics/zbar'

def generate_launch_description():
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )
    # load ros config
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(str(Path(
        #         get_package_share_directory('stingray_core_launch'), 'auv.launch.py')))),
        transition_srv_arg,
        zbar_topic_arg,
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            # respawn=True,
            # respawn_delay=1,
        ),

        # Node(
        #     package='stingray_movement',
        #     executable='twist_action_server',
        #     name='twist_action_server',
        #     # respawn=True,
        #     # respawn_delay=1,
        # ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
        ),
        Node(
            package='stingray_missions',
            executable='qr_trigger_node',
            name='qr_trigger_node',
            parameters=[
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'zbar_topic': LaunchConfiguration("zbar_topic")}
            ]
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader',
            remappings=[
                ('/image', '/stingray/topics/camera/front'),
                ('/barcode', LaunchConfiguration("zbar_topic")),
            ]
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
