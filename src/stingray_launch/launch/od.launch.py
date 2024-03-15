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
   

    # load ros config
    return LaunchDescription([
        front_camera_topic_arg,
        front_camera_path_arg,
        image_topic_list_arg,
        set_enable_object_detection_srv_arg,
        weights_pkg_name_arg,
        debug_arg,
        
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
