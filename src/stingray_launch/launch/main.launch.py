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
    # missions
    mission_package_names_arg = DeclareLaunchArgument(
        "mission_package_names", default_value='stingray_missions stingray_missions'
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

    # devices
    device_action_arg = DeclareLaunchArgument(
        "device_action", default_value='/stingray/actions/device'
    )
    device_state_array_topic_arg = DeclareLaunchArgument(
        "device_state_array_topic", default_value='/stingray/topics/device_state_array'
    )

    # object detection
    image_topic_list_arg = DeclareLaunchArgument(
        "image_topic_list", default_value='/stingray/topics/front_camera'
    )
    set_enable_object_detection_srv_arg = DeclareLaunchArgument(
        "set_enable_object_detection_srv", default_value='/stingray/services/set_twist'
    )
    weights_pkg_name_arg = DeclareLaunchArgument(
        "weights_pkg_name", default_value='stingray_object_detection'
    )
    debug_arg = DeclareLaunchArgument(
        "debug", default_value='True'
    )

    # core services
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )
    set_device_srv_arg = DeclareLaunchArgument(
        "set_device_srv", default_value='/stingray/services/set_device'
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
        mission_package_names_arg,
        transition_srv_arg,
        zbar_topic_arg,
        twist_action_arg,
        uv_state_topic_arg,
        set_twist_srv_arg,
        device_action_arg,
        device_state_array_topic_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        enable_thrusters_srv_arg,
        set_device_srv_arg,
        image_topic_list_arg,
        set_enable_object_detection_srv_arg,
        weights_pkg_name_arg,
        debug_arg,

        # missions
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            parameters=[
                {'mission_package_names': LaunchConfiguration("mission_package_names")},
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'twist_action': LaunchConfiguration("twist_action")},
                {'device_action': LaunchConfiguration("device_action")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv")},
                {'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv")},
                {'set_enable_object_detection_srv': LaunchConfiguration("set_enable_object_detection_srv")},
            ],
            respawn=True,
            respawn_delay=1,
        ),

        # qr trigger
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

        # qr reader
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

        # movement
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

        # devices
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

        # object detection        
        Node(
            package='stingray_object_detection',
            executable='yolov5_detector',
            name='yolov5_detector',
            parameters=[
                {'weights_pkg_name': LaunchConfiguration("weights_pkg_name")},
                {'image_topic_list': LaunchConfiguration("image_topic_list")},
                {'set_enable_object_detection_srv': LaunchConfiguration("set_enable_object_detection_srv")},
                {'debug': LaunchConfiguration("debug")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
