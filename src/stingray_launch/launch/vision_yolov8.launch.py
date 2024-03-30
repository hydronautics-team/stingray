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
    set_enable_object_detection_srv_arg = DeclareLaunchArgument(
        "set_enable_object_detection_srv", default_value='/stingray/services/set_enable_object_detection'
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
        set_enable_object_detection_srv_arg,
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
                {'set_enable_object_detection_srv': LaunchConfiguration(
                    "set_enable_object_detection_srv")},
                {'debug': LaunchConfiguration("debug")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
