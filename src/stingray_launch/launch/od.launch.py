from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # object detection
    image_topic_list_arg = DeclareLaunchArgument(
        "image_topic_list", default_value='[/stingray/topics/camera]'
    )
    camera_info_topic_list_arg = DeclareLaunchArgument(
        "camera_info_topic_list", default_value='[/stingray/topics/camera/camera_info]'
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
        image_topic_list_arg,
        camera_info_topic_list_arg,
        enable_object_detection_topic_arg,
        weights_pkg_name_arg,
        bbox_attrs_pkg_name_arg,
        debug_arg,

        # object detection
        Node(
            package='stingray_object_detection',
            executable='yolov8_detector',
            name='yolov8_detector',
            parameters=[
                {'weights_pkg_name': LaunchConfiguration("weights_pkg_name")},
                {'bbox_attrs_pkg_name': LaunchConfiguration("bbox_attrs_pkg_name")},
                {'image_topic_list': LaunchConfiguration("image_topic_list")},
                {'camera_info_topic_list': LaunchConfiguration("camera_info_topic_list")},
                {'enable_object_detection_topic': LaunchConfiguration(
                    "enable_object_detection_topic")},
                {'debug': LaunchConfiguration("debug")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
