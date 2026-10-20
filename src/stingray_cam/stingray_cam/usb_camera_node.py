#!/usr/bin/env python3
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


class UsbCameraNode(Node):
    """Publish a V4L2 camera and its calibration as ROS image topics."""

    def __init__(self):
        super().__init__('front_usb_camera')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_topic', '/stingray/topics/camera/front')
        self.declare_parameter(
            'camera_info_topic',
            '/stingray/topics/camera/front/camera_info',
        )
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('frame_id', 'front_camera_optical_frame')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30.0)

        device = self.get_parameter('video_device').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        framerate = self.get_parameter('framerate').value

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, framerate)
        if not self.capture.isOpened():
            raise RuntimeError(f'Unable to open USB camera: {device}')

        self.bridge = CvBridge()
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_info = self._load_camera_info(
            self.get_parameter('camera_info_url').value,
            width,
            height,
        )
        self.image_publisher = self.create_publisher(
            Image,
            self.get_parameter('image_topic').value,
            qos_profile_sensor_data,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / framerate, self._publish_frame)
        self.get_logger().info(
            f'Publishing {device} at {width}x{height}@{framerate:g} FPS')

    def _load_camera_info(self, url, width, height):
        message = CameraInfo()
        message.width = width
        message.height = height
        if not url:
            self.get_logger().warning(
                'No camera calibration configured; distance estimates '
                'will be unavailable')
            return message

        path = Path(url.removeprefix('file://'))
        try:
            with path.open(encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file)
            message.width = config['image_width']
            message.height = config['image_height']
            message.distortion_model = config['distortion_model']
            message.d = config['distortion_coefficients']['data']
            message.k = config['camera_matrix']['data']
            message.r = config['rectification_matrix']['data']
            message.p = config['projection_matrix']['data']
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            raise RuntimeError(
                f'Unable to load camera calibration {path}: {error}') from error
        return message

    def _publish_frame(self):
        success, frame = self.capture.read()
        if not success:
            self.get_logger().warning(
                'Unable to read a frame from the USB camera',
                throttle_duration_sec=5.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.image_publisher.publish(image)
        self.camera_info_publisher.publish(self.camera_info)

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
