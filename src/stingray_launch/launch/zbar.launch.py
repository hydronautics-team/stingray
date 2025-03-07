from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_config_dir = Path(get_package_share_directory('stingray_cam')) / "configs"
    camera_calibration_path = calibration_config_dir / "camera.yaml"
    # front camera
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic", default_value='/stingray/topics/camera'
    )
    camera_path_arg = DeclareLaunchArgument(
        "camera_path", default_value='/dev/video0'
    )
    camera_calibration_path_arg = DeclareLaunchArgument(
        "camera_calibration_path", default_value=str(camera_calibration_path)
    )

    # zbar camera
    zbar_camera_topic_arg = DeclareLaunchArgument(
        "zbar_camera_topic", default_value='/stingray/topics/camera'
    )

    # zbar
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )


    # load ros config
    return LaunchDescription([
        camera_topic_arg,
        camera_path_arg,
        camera_calibration_path_arg,
        zbar_camera_topic_arg,
        zbar_topic_arg,

        # front camera
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node',
            remappings=[
                ('/image_raw', LaunchConfiguration("camera_topic")),
            ],
            parameters=[
                {'video_device': LaunchConfiguration("camera_path")},
                {'params-file': LaunchConfiguration("camera_calibration_path")},
                # {'image_width': 640},
                # {'image_height': 480},
                # {'camera_name': 'camera'},
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
                ('/image', LaunchConfiguration("zbar_camera_topic")),
                ('/barcode', LaunchConfiguration("zbar_topic")),
            ],
            respawn=True,
            respawn_delay=1,
        ),
    ])
