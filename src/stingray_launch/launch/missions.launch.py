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
    # object detection
    bbox_array_topic_arg = DeclareLaunchArgument(
        "bbox_array_topic", default_value='/stingray/topics/camera/front/bbox_array'
    )

    # missions
    mission_package_names_arg = DeclareLaunchArgument(
        "mission_package_names", default_value='[stingray_missions]'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )

    # object detection
    enable_object_detection_topic_arg = DeclareLaunchArgument(
        "enable_object_detection_topic", default_value='/stingray/topics/enable_object_detection'
    )

    # movement
    twist_action_arg = DeclareLaunchArgument(
        "twist_action", default_value='/stingray/actions/twist'
    )
    bbox_centering_twist_action_arg = DeclareLaunchArgument(
        "bbox_centering_twist_action", default_value='/stingray/actions/bbox_centering_twist'
    )
    bbox_search_twist_action_arg = DeclareLaunchArgument(
        "bbox_search_twist_action", default_value='/stingray/actions/bbox_search_twist'
    )
    hydroacoustic_centering_twist_action_arg = DeclareLaunchArgument(
        "hydroacoustic_centering_twist_action", default_value='/stingray/actions/hydroacoustic_centering_twist'
    )
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )

    # devices
    device_action_arg = DeclareLaunchArgument(
        "device_action", default_value='/stingray/actions/device'
    )

    # core
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    enable_thrusters_srv_arg = DeclareLaunchArgument(
        "enable_thrusters_srv", default_value='/stingray/services/enable_thrusters'
    )

    # load ros config
    return LaunchDescription([
        bbox_array_topic_arg,
        mission_package_names_arg,
        transition_srv_arg,
        enable_object_detection_topic_arg,
        twist_action_arg,
        bbox_centering_twist_action_arg,
        hydroacoustic_centering_twist_action_arg,
        bbox_search_twist_action_arg,
        device_action_arg,
        uv_state_topic_arg,
        set_twist_srv_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        enable_thrusters_srv_arg,

        # missions
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            parameters=[
                {'mission_package_names': LaunchConfiguration("mission_package_names")},
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'twist_action': LaunchConfiguration("twist_action")},
                {'bbox_centering_twist_action': LaunchConfiguration("bbox_centering_twist_action")},
                {'bbox_search_twist_action': LaunchConfiguration("bbox_search_twist_action")},
                {'hydroacoustic_centering_twist_action': LaunchConfiguration("hydroacoustic_centering_twist_action")},
                {'device_action': LaunchConfiguration("device_action")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv")},
                {'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv")},
                {'enable_object_detection_topic': LaunchConfiguration("enable_object_detection_topic")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(str(Path(
        #         get_package_share_directory('sauvc_launch'), 'pinger.launch.py')))
        # ),
        
    ])
