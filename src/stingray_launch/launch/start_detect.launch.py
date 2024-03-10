import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            #sudo apt-get install ros-<ros2-distro>-usb-cam
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            arguments=['--ros-args', '-p', 'video_device:= "/dev/video0"']
        ),
        Node(
            package='stingray_object_detection',
            executable='yolov8_detector',
            name='yolov8_detector'
        )
        # Node(
        #     package='stingray_object_detection',
        #     executable='yolo_detector',
        #     name='yolo_detector',
        #     parameters=[
        #         {"weights_pkg_name": "stingray_object_detection"},
        #         {'image_topic_list': 'usb_cam_node '}]
        #         # {'debug': ''}]
        # )
        # Node(
        #     package='ros2_video_streamer',
        #     executable='camera_simulator',
        #     name='camera_simulator_node',
        #     arguments=['--type', 'video', '--path', '/home/shakuevda/Desktop/test.webm', '--loop']
        #     )

    ])