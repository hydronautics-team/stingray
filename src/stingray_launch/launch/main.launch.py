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


def generate_launch_description():
    # camera
    front_camera_topic_arg = DeclareLaunchArgument(
        "front_camera_topic", default_value='/stingray/topics/front_camera'
    )
    front_camera_path_arg = DeclareLaunchArgument(
        "front_camera_path", default_value='/dev/video0'
    )

    # missions
    package_names_arg = DeclareLaunchArgument(
        "package_names", default_value='[stingray_missions stingray_missions]'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )

    # movement
    twist_action_arg = DeclareLaunchArgument(
        "twist_action", default_value='/stingray/actions/twist'
    )
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )

    # devices
    device_action_arg = DeclareLaunchArgument(
        "device_action", default_value='/stingray/actions/device'
    )
    device_state_array_topic_arg = DeclareLaunchArgument(
        "device_state_array_topic", default_value='/stingray/topics/device_state_array'
    )
    set_device_srv_arg = DeclareLaunchArgument(
        "set_device_srv", default_value='/stingray/services/set_device'
    )

    # core
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )

    # load ros config
    return LaunchDescription([
        front_camera_topic_arg,
        front_camera_path_arg,
        package_names_arg,
        transition_srv_arg,
        zbar_topic_arg,
        twist_action_arg,
        uv_state_topic_arg,
        set_twist_srv_arg,
        device_action_arg,
        device_state_array_topic_arg,
        set_device_srv_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            parameters=[
                # {'package_names': LaunchConfiguration("package_names")},
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'twist_action': LaunchConfiguration("twist_action")},
                {'device_action': LaunchConfiguration("device_action")},
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
            parameters=[
                {'twist_action': LaunchConfiguration("twist_action")},
                {'uv_state_topic': LaunchConfiguration("uv_state_topic")},
                {'set_twist_srv': LaunchConfiguration("set_twist_srv")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='stingray_devices',
            executable='device_action_server',
            name='device_action_server',
            parameters=[
                {'device_action': LaunchConfiguration("device_action")},
                {'device_state_array_topic': LaunchConfiguration(
                    "device_state_array_topic")},
                {'set_device_srv': LaunchConfiguration("set_device_srv")},
            ],
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
            parameters=[
                {'video_device': LaunchConfiguration("front_camera_path")},
                # {'camera_name': "/front"},
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
