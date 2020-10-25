#!/usr/bin/env python

import cv2 as cv
import json
import os
import numpy as np
import time
import os
from random import randint
from itertools import groupby

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from stingray_vision_msgs.msg import Object
from stingray_vision_msgs.msg import ObjectsArray
from sensor_msgs.msg import Image


class ObjectDetector:
    def __init__(self, input_image_topic, confidence, enable_output_image_publishing, package_name_with_net):
        # get node name
        node_name = rospy.get_name()
        rospy.loginfo("{} node initializing".format(node_name))
        # get paths
        rospack = rospkg.RosPack()
        path = rospack.get_path(package_name_with_net)
        weights_path = os.path.sep.join(
            [path, "net", "frozen_inference_graph.pb"])
        labels_path = os.path.sep.join([path, "net", "labels.json"])
        config_path = os.path.sep.join([path, "net", "opencv_graph.pbtxt"])
        self.confidence = confidence
        # read labels
        with open(labels_path) as json_file:
            self.labels = json.loads(json_file.read())["labels"]
        # random colors
        self.colors = {}
        for label in self.labels:
            color = (randint(0, 255), randint(0, 255), randint(0, 255))
            self.colors[label["name"]] = color

        # init cv_bridge
        self.bridge = CvBridge()
        # load our NET from disk
        rospy.loginfo("Loading neural network")
        self.cvNet = cv.dnn.readNetFromTensorflow(weights_path, config_path)

        # ROS Topic names
        objects_array_topic = "{}/objects".format(node_name)
        output_image_topic = "{}/image".format(node_name)

        self.enable_output_image_publishing = enable_output_image_publishing

        # publishers
        self.objects_array_pub = rospy.Publisher(
            objects_array_topic, ObjectsArray, queue_size=10)
        if self.enable_output_image_publishing:
            self.image_pub = rospy.Publisher(
                output_image_topic, Image, queue_size=1)

        # subscribers
        self.image_sub = rospy.Subscriber(
            input_image_topic, Image, self.callback, queue_size=1)

    def callback(self, data):
        try:
            # convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # detect our objects
            dnn_objects = self.detector(cv_image)
            # publish objects
            # init msg
            self.object_msg = Object()
            self.objects_array_msg = ObjectsArray()
            for index, dnn_object in enumerate(dnn_objects):
                self.object_msg.name = dnn_object["name"].encode('utf-8')
                self.object_msg.confidence = dnn_object["confidence"]
                top_left_x = int(dnn_object['box'][0])
                self.object_msg.top_left_x = top_left_x
                top_left_y = int(dnn_object['box'][1])
                self.object_msg.top_left_y = top_left_y
                bottom_right_x = int(dnn_object['box'][2])
                self.object_msg.bottom_right_x = bottom_right_x
                bottom_right_y = int(dnn_object['box'][3])
                self.object_msg.bottom_right_y = bottom_right_y
                self.objects_array_msg.objects.append(self.object_msg)
            self.objects_array_pub.publish(self.objects_array_msg)

            if self.enable_output_image_publishing:
                # draw bounding boxes
                dnn_cv_image = self.draw(cv_image, dnn_objects)
                # convert cv image into ros format
                ros_image = self.bridge.cv2_to_imgmsg(dnn_cv_image, "bgr8")
                # publish output image
                self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)

    def detector(self, img):
        # construct a blob from the input image and then perform a
        # forward pass, giving us the bounding box
        # coordinates of the objects in the image
        self.cvNet.setInput(cv.dnn.blobFromImage(
            img, size=(300, 300), swapRB=True, crop=False))
        start = time.time()
        cvOut = self.cvNet.forward()
        end = time.time()
        objects = []
        rows = img.shape[0]
        cols = img.shape[1]
        # go through all detected objects
        for i in range(0, cvOut.shape[2]):
            # check confidence
            classID = int(cvOut[0, 0, i, 1])
            confidence = cvOut[0, 0, i, 2]
            # filter out weak predictions by ensuring the detected probability
            # is greater than the minimum probability
            if confidence > self.confidence:
                object_ = {}
                object_["name"] = self.labels[classID-1]["name"]
                object_["confidence"] = confidence
                # clone our original image so we can draw on it
                clone = img.copy()
                # scale the bounding box coordinates back relative to the
                # size of the image and then compute the width and the height
                # of the bounding box
                box = cvOut[0, 0, i, 3:7] * np.array([cols, rows, cols, rows])
                object_["box"] = box.astype("int")
                objects.append(object_)
        return self.deleteMultipleObjects(objects)

    def deleteMultipleObjects(self, objects):
        # filter founded objects
        # group by name and sort
        groups = [(group_name, list(group)) for group_name, group in groupby(
            sorted(objects, key=lambda label: label['name']), lambda label: label['name'])]
        filtered_objects = []
        # go through groups and pop unnecessary items
        for group in groups:
            object_count_from_json = list(
                filter(lambda label: label['name'] == group[0], self.labels))[0]["count"]
            del group[1][object_count_from_json:]
            filtered_objects += group[1]
        return filtered_objects

    def draw(self, img, objects):
        for dnn_object in objects:
            # get box coordinates
            top_left_x = dnn_object['box'][0]
            top_left_y = dnn_object['box'][1]
            bottom_right_x = dnn_object['box'][2]
            bottom_right_y = dnn_object['box'][3]
            center_x = int(top_left_x + (bottom_right_x - top_left_x)/2)
            center_y = int(top_left_y + (bottom_right_y - top_left_y)/2)
            # draw rectangle and center point
            cv.rectangle(img, (top_left_x, top_left_y),
                         (bottom_right_x, bottom_right_y), self.colors[dnn_object["name"]], thickness=2)
            cv.line(img, (center_x - 5, center_y - 5),
                    (center_x + 5, center_y + 5), self.colors[dnn_object["name"]], thickness=1)
            cv.line(img, (center_x + 5, center_y - 5),
                    (center_x - 5, center_y + 5), self.colors[dnn_object["name"]], thickness=1)
            # draw the predicted label and associated probability of the
            # instance segmentation on the image
            if (top_left_y < 15):
                text_name = "{}".format(dnn_object["name"])
                cv.putText(img, text_name, (top_left_x + 3, top_left_y + 15),
                           cv.FONT_HERSHEY_DUPLEX, 0.5, self.colors[dnn_object["name"]], 1)
                text_confidence = "{:.2f}".format(dnn_object["confidence"])
                cv.putText(img, text_confidence, (top_left_x + 3, top_left_y + 35),
                           cv.FONT_HERSHEY_DUPLEX, 0.5, self.colors[dnn_object["name"]], 1)
            else:
                text_name = "{}".format(dnn_object["name"])
                cv.putText(img, text_name, (top_left_x + 3, top_left_y - 5),
                           cv.FONT_HERSHEY_DUPLEX, 0.5, self.colors[dnn_object["name"]], 1)
                text_confidence = "{:.2f}".format(dnn_object["confidence"])
                cv.putText(img, text_confidence, (top_left_x + 3, top_left_y + 15),
                           cv.FONT_HERSHEY_DUPLEX, 0.5, self.colors[dnn_object["name"]], 1)
        return img


if __name__ == '__main__':
    rospy.init_node('object_detector')
    # parameters
    input_image_topic = rospy.get_param('~input_image_topic')
    dnn_confidence_threshold = rospy.get_param('~dnn_confidence_threshold')
    enable_output_image_publishing = rospy.get_param(
        '~enable_output_image_publishing')
    package_name_with_net = rospy.get_param('~package_name_with_net')
    try:
        ot = ObjectDetector(input_image_topic, dnn_confidence_threshold,
                            enable_output_image_publishing, package_name_with_net)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down {} node".format(rospy.get_name()))
