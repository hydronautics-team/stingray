from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # object detection
    bbox_array_topic_arg = DeclareLaunchArgument(
        "bbox_array_topic", default_value='/stingray/topics/camera/bbox_array'
    )
    target_close_thresh_arg = DeclareLaunchArgument(
        "target_close_thresh", default_value='2.5'
    )

    # missions
    mission_package_names_arg = DeclareLaunchArgument(
        "mission_package_names", default_value='[stingray_missions]'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
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

    # additional
    debug_arg = DeclareLaunchArgument(
        "debug", default_value='True'
    )

    return LaunchDescription([
        bbox_array_topic_arg,
        target_close_thresh_arg,
        mission_package_names_arg,
        transition_srv_arg,
        enable_object_detection_topic_arg,
        zbar_topic_arg,
        twist_action_arg,
        bbox_centering_twist_action_arg,
        uv_state_topic_arg,
        set_twist_srv_arg,
        device_action_arg,
        device_state_array_topic_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        enable_thrusters_srv_arg,
        set_device_srv_arg,
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
                {'bbox_centering_twist_action': LaunchConfiguration("bbox_centering_twist_action")},
                {'device_action': LaunchConfiguration("device_action")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv")},
                {'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv")},
                {'enable_object_detection_topic': LaunchConfiguration("enable_object_detection_topic")},
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
        Node(
            package='stingray_movement',
            executable='bbox_centering_twist_action_server',
            name='bbox_centering_twist_action_server',
            parameters=[
                {'bbox_centering_twist_action': LaunchConfiguration("bbox_centering_twist_action")},
                {'uv_state_topic': LaunchConfiguration("uv_state_topic")},
                {'bbox_array_topic': LaunchConfiguration("bbox_array_topic")},
                {'set_twist_srv': LaunchConfiguration("set_twist_srv")},
                {'target_close_thresh': LaunchConfiguration("target_close_thresh")},
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
    ])
