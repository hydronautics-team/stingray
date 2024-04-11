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
        self.declare_parameter('source_topic')
        self.declare_parameter('output_width')
        self.declare_parameter('output_height')
        self.declare_parameter('output_fps')
        self.declare_parameter('output_format')
        self.declare_parameter('record_dir')


class VideoFrames:
    def __init__(self, image_topic, target_x, target_y, target_w, target_h, working_node: VideoRecorderNode):
        self.node = working_node
        self.image_sub = self.node.create_subscription(Image, image_topic, self.callback_image, 1)
        self.bridge = CvBridge()
        self.frames = []
        self.target_x, self.target_y, self.target_w, self.target_h = target_x, target_y, target_w, target_h

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().error('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
            return

        self.frames.append((time.time(), cv_image))

    def get_latest(self, at_time, remove_older=True):
        fs = [x for x in self.frames if x[0] <= at_time]
        if len(fs) == 0:
            return None

        f = fs[-1]
        if remove_older:
            self.frames = self.frames[len(fs) - 1:]

        return f[1]


class VideoRecorder:
    def __init__(self, output_width, output_height, output_fps,
                 output_format, output_path, source, node: VideoRecorderNode):
        self.node = node
        self.frame_wrappers = []
        self.start_time = -1
        self.end_time = -1
        self.rate = None
        self.pub_img = None
        self.bridge = CvBridge()

        self.fps = output_fps
        self.interval = 1.0 / self.fps
        self.output_width = output_width
        self.output_height = output_height

        if opencv_version() == 2:
            fourcc = cv2.cv.FOURCC(*output_format)
        elif opencv_version() == 3 or opencv_version() == 4:
            fourcc = cv2.VideoWriter_fourcc(*output_format)
        else:
            raise

        self.output_path = output_path

        if self.output_path:
            self.video_writer = cv2.VideoWriter(output_path, fourcc, output_fps, (output_width, output_height))
        else:
            self.video_writer = None

        vf = VideoFrames(source, target_x=0, target_y=0, target_w=640, target_h=480, working_node=self.node)

        self.frame_wrappers.append(vf)

    def start_record(self):
        self.rate = self.node.create_rate(self.fps)
        self.start_time = time.time()
        curr_time = self.start_time
        while self.end_time < 0 or curr_time <= self.end_time:
            try:
                canvas = np.zeros((self.output_height, self.output_width, 3), np.uint8)

                for frame_w in self.frame_wrappers:
                    f = frame_w.get_latest(at_time=curr_time)
                    if f is None:
                        continue

                    resized = cv2.resize(f, (frame_w.target_w, frame_w.target_h))
                    canvas[frame_w.target_y:frame_w.target_y + frame_w.target_h,
                    frame_w.target_x:frame_w.target_x + frame_w.target_w] = resized
                    # rospy.sleep(0.01)

                if self.video_writer:
                    self.video_writer.write(canvas)
                if self.pub_img:
                    try:
                        self.pub_img.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))
                    except CvBridgeError as e:
                        self.node.get_logger().error('cvbridgeerror, e={}'.format(str(e)))
                        pass
                self.rate.sleep()

                if not self.node.context.ok() and self.end_time < 0:
                    self.terminate()

                while curr_time + self.interval > time.time():
                    self.rate.sleep()

                curr_time += self.interval
            except KeyboardInterrupt:
                self.terminate()
                continue

        if self.video_writer:
            self.video_writer.release()

    def terminate(self):
        self.node.get_logger().info("[ros-video-recorder] Video Saved. path={}".format(self.output_path))
        self.end_time = time.time()

def main(*args, **kwargs):
    rclpy.init()
    node = VideoRecorderNode("stingray_video_recorder")

    # parameters
    source_topic = node.get_parameter('source_topic').value
    output_width = int(node.get_parameter_or('output_width', rclpy.Parameter(
        'int', rclpy.Parameter.Type.INTEGER, 640)).value)
    output_height = int(node.get_parameter_or('output_height', rclpy.Parameter(
        'int', rclpy.Parameter.Type.INTEGER, 480)).value)
    output_fps = int(node.get_parameter_or('output_fps', rclpy.Parameter(
        'int', rclpy.Parameter.Type.INTEGER, 30)).value)
    output_format = node.get_parameter_or('output_format', rclpy.Parameter(
        'str', rclpy.Parameter.Type.STRING, 'h264')).value

    output_path = node.get_parameter_or('record_dir', rclpy.Parameter(
        'str', rclpy.Parameter.Type.STRING, './records/')).value

    if output_path[-1] != '/':
        output_path += '/'
    if not os.path.isdir(output_path):
        os.makedirs(output_path)

    path_part = source_topic.replace('~', '').replace('/', '_')
    if path_part[0] == '_':
        path_part = path_part[1:]
    output_path += path_part + '_' + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.avi'

    ft = VideoRecorder(output_width, output_height, output_fps, output_format, output_path, source_topic, node=node)

    # recording.
    try:
        ft.start_record()
    except KeyboardInterrupt:
        node.get_logger().error("[ros-video-recorder] Shutting down+")

    ft.terminate()


if __name__ == '__main__':
    main()
