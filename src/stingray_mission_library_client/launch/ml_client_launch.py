from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stingray_mission_library_client',
            executable='ml_client',
            name='ml_client'
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            remappings=[
                ('/image_raw', '/image'),
            ]
        )
    ])
