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

    # object detection
    image_topic_list_arg = DeclareLaunchArgument(
        "image_topic_list", default_value='[/stingray/topics/camera]'
    )
    enable_object_detection_topic_arg = DeclareLaunchArgument(
        "enable_object_detection_topic", default_value='/stingray/topics/enable_object_detection'
    )
    weights_pkg_name_arg = DeclareLaunchArgument(
        "weights_pkg_name", default_value='stingray_object_detection'
    )
    bbox_attrs_pkg_name_arg = DeclareLaunchArgument(
        "bbox_attrs_pkg_name", default_value='stingray_object_detection'
    )
    debug_arg = DeclareLaunchArgument(
        "debug", default_value='True'
    )

    # load ros config
    return LaunchDescription([
        zbar_camera_topic_arg,
        zbar_topic_arg,
        image_topic_list_arg,
        enable_object_detection_topic_arg,
        weights_pkg_name_arg,
        bbox_attrs_pkg_name_arg,
        debug_arg,

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

        # object detection
        Node(
            package='stingray_object_detection',
            executable='yolov8_detector',
            name='yolov8_detector',
            parameters=[
                {'weights_pkg_name': LaunchConfiguration("weights_pkg_name")},
                {'bbox_attrs_pkg_name': LaunchConfiguration("bbox_attrs_pkg_name")},
                {'image_topic_list': LaunchConfiguration("image_topic_list")},
                {'enable_object_detection_topic': LaunchConfiguration(
                    "enable_object_detection_topic")},
                {'debug': LaunchConfiguration("debug")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
