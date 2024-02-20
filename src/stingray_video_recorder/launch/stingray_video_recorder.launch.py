from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import EnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    SetEnvironmentVariable(name='PATH', value=[EnvironmentVariable('PATH'), '/stingray_video_records'])
    device = DeclareLaunchArgument(
        "device", default_value=TextSubstitution(text="/dev/video0")
    )
    fps = DeclareLaunchArgument(
        "fps", default_value=TextSubstitution(text="25")
    )
    stream = DeclareLaunchArgument(
        "stream", default_value=TextSubstitution(text="0")
    )
    record = DeclareLaunchArgument(
        "record", default_value=TextSubstitution(text="1")
    )

    # #TODO need migration to ros2 for usb_cam
    # device_camera = Node(
    #         package='usb_cam',
    #         namespace='usb_cam',
    #         executable='usb_cam',
    #         name='device_camera',
    #         respawn=1,
    #         parameters=[{
    #             "video_device": LaunchConfiguration("video_device", default=device.default_value),
    #             "image_width": LaunchConfiguration("image_width", default="640"),
    #             "image_height": LaunchConfiguration("image_height", default="480"),
    #             "pixel_format": LaunchConfiguration("pixel_format", default="yuyv"),
    #             "framerate": LaunchConfiguration("framerate", default=fps.default_value),
    #             "camera_frame_id": LaunchConfiguration("camera_frame_id", default="device_camera"),
    #             "camera_name": LaunchConfiguration("camera_name", default="device_camera"),
    #             "io_method": LaunchConfiguration("io_method", default="mmap"),
    #         }]
    #
    #     )

    video_recorder = Node(
            package='stingray_video_recorder',
            executable='video_recorder',
            name='stingray_recorder',
            respawn=True,
            # parameters=[{
            #     "source_topic": LaunchConfiguration('source_topic', default="/device_camera/image_raw"),
            #     "output_width": LaunchConfiguration("output_width", default='640'),
            #     "output_height": LaunchConfiguration("output_height", default='480'),
            #     "output_fps": LaunchConfiguration("output_fps", default=fps.default_value),
            #     "output_format": LaunchConfiguration("output_format", default='h264'),
            #     "record_dir": LaunchConfiguration("record_dir", default=EnvironmentVariable('HOME'))
            # }],
            condition=LaunchConfigurationEquals('record', '1')
        )
    usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            remappings=[
                ('/image_raw', 'device_camera/image_raw'),
            ]
        )
    # video_server = Node(
    #     name='video_server',
    #     package='web_video_server',
    #     executable='web_video_server',
    #     respawn=1,
    #     parameters=[{
    #         "server_threads": LaunchConfiguration("server_threads", default='4'),
    #         "ros_threads": LaunchConfiguration("ros_threads", default='10'),
    #
    #     }],
    #     condition=LaunchConfigurationEquals('stream', '1')
    # )

    return LaunchDescription([
        device,
        fps,
        stream,
        record,
        usb_cam,
        # device_camera,
        video_recorder,
        # video_server,
    ])
