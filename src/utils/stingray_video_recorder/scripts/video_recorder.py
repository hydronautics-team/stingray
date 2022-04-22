#!/usr/bin/env python

from __future__ import print_function
import cv2
import numpy as np
import datetime
import time
import os

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def opencv_version():
    v = cv2.__version__.split('.')[0]
    if v == '2':
        return 2
    elif v == '3':
        return 3
    elif v == '4':
        return 4
    raise Exception('opencv version can not be parsed. v={}'.format(v))


class VideoFrames:
    def __init__(self, image_topic, target_x, target_y, target_w, target_h):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.frames = []
        self.target_x, self.target_y, self.target_w, self.target_h = target_x, target_y, target_w, target_h

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
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
    def __init__(self, output_width, output_height, output_fps, output_format, output_path, source):
        self.frame_wrappers = []
        self.start_time = -1
        self.end_time = -1
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

        vf = VideoFrames(source, target_x=0, target_y=0, target_w=640, target_h=480)

        self.frame_wrappers.append(vf)

    def start_record(self):
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
                        rospy.logerr('cvbridgeerror, e={}'.format(str(e)))
                        pass
                rospy.sleep(0.01)

                if rospy.is_shutdown() and self.end_time < 0:
                    self.terminate()

                while curr_time + self.interval > time.time():
                    rospy.sleep(self.interval)

                curr_time += self.interval
            except KeyboardInterrupt:
                self.terminate()
                continue

        if self.video_writer:
            self.video_writer.release()

    def terminate(self):
        rospy.loginfo("[ros-video-recorder] Video Saved. path={}".format(self.output_path))
        self.end_time = time.time()


if __name__ == '__main__':
    rospy.init_node('stingray_video_recorder', anonymous=True)

    # parameters
    source_topic = rospy.get_param('~source_topic')
    output_width = int(rospy.get_param('~output_width', '640'))
    output_height = int(rospy.get_param('~output_height', '480'))
    output_fps = int(rospy.get_param('~output_fps', '30'))
    output_format = rospy.get_param('~output_format', 'h264')
    output_path = rospy.get_param('~record_dir', './records/')

    if output_path[-1] != '/':
        output_path += '/'
    if not os.path.isdir(output_path):
        os.makedirs(output_path)

    path_part = source_topic.replace('~', '').replace('/', '_')
    if path_part[0] == '_':
        path_part = path_part[1:]
    output_path += path_part + '_' + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + '.avi'

    ft = VideoRecorder(output_width, output_height, output_fps, output_format, output_path, source_topic)

    # recording.
    try:
        ft.start_record()
    except KeyboardInterrupt:
        rospy.logerr("[ros-video-recorder] Shutting down+")

    ft.terminate()