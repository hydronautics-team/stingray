#!/usr/bin/env python3

import cv2
import numpy as np
import datetime
import time
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# TODO cleanup and refactor

def opencv_version():
    v = cv2.__version__.split('.')[0]
    if v == '2':
        return 2
    elif v == '3':
        return 3
    elif v == '4':
        return 4
    raise Exception('opencv version can not be parsed. v={}'.format(v))


class VideoRecorderNode(Node):
    def __init__(self, name):
        super(VideoRecorderNode, self).__init__(name)
        self.declare_parameter('source_topic', '/stingray/topics/camera')
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        self.declare_parameter('output_fps', 15)
        self.declare_parameter('output_format', 'h264')
        self.declare_parameter('record_dir', "./records/")

        self.bridge = CvBridge()


        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        output_height = self.get_parameter('output_height').get_parameter_value().integer_value
        output_fps = self.get_parameter('output_fps').get_parameter_value().integer_value
        output_format = self.get_parameter('output_format').get_parameter_value().string_value
        output_path = self.get_parameter('record_dir').get_parameter_value().string_value

        if output_path[-1] != '/':
            output_path += '/'
        if not os.path.isdir(output_path):
            os.makedirs(output_path)

        path_part = source_topic.replace('~', '').replace('/', '_')
        if path_part[0] == '_':
            path_part = path_part[1:]
        output_path += path_part + '_' + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.avi'


        self.interval = 1.0 / output_fps

        if opencv_version() == 2:
            fourcc = cv2.cv.FOURCC(*output_format)
        elif opencv_version() == 3 or opencv_version() == 4:
            fourcc = cv2.VideoWriter_fourcc(*output_format)
        else:
            raise

        if output_path:
            self.video_writer = cv2.VideoWriter(output_path, fourcc, output_fps, (output_width, output_height))
        else:
            self.video_writer = None

        self.image_sub = self.create_subscription(Image, source_topic, self.callback_image, 1)

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
            return
        self.video_writer.write(cv_image)


def main(*args, **kwargs):
    rclpy.init()
    node = VideoRecorderNode("stingray_video_recorder")

    rclpy.spin(node)

    node.video_writer.release()
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
