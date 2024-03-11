import os
from pathlib import Path

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
twist_action = '/stingray/topics/zbar'
reset_imu_srv = '/stingray/topics/zbar'
set_stabilization_srv = '/stingray/topics/zbar'


def generate_launch_description():
    front_camera_topic_arg = DeclareLaunchArgument(
        "front_camera_topic", default_value='/stingray/topics/front_camera'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )
    twist_action_arg = DeclareLaunchArgument(
        "twist_action", default_value='/stingray/actions/twist'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    # load ros config
    return LaunchDescription([
        
        front_camera_topic_arg,
        transition_srv_arg,
        zbar_topic_arg,
        twist_action_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            parameters=[
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'twist_action': LaunchConfiguration("twist_action")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'set_stabilization_srv': LaunchConfiguration(
                    "set_stabilization_srv")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='stingray_movement',
            executable='twist_action_server',
            name='twist_action_server',
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='stingray_missions',
            executable='qr_trigger_node',
            name='qr_trigger_node',
            parameters=[
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'zbar_topic': LaunchConfiguration("zbar_topic")}
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader',
            remappings=[
                ('/image', LaunchConfiguration("front_camera_topic")),
                ('/barcode', LaunchConfiguration("zbar_topic")),
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            remappings=[
                ('/image_raw', LaunchConfiguration("front_camera_topic")),
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
        ),
    ])
