#!/usr/bin/env python3


import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from object_detection_msgs.msg import Object
from object_detection_msgs.msg import ObjectsArray
from sensor_msgs.msg import Image
from itertools import groupby
import time
import os
import sys
import cv2
import torch
import numpy as np

# sys.path.append("./yolov5")
sys.path.insert(1, os.path.join(rospkg.RosPack().get_path("object_detection"), "scripts/yolov5"))

from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                                  increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from yolov5.utils.plots import Annotator, colors, save_one_box
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.augmentations import letterbox



class YoloDetector:
    def __init__(self,
                 weights_pkg_path,
                 input_image_topic,
                 confidence_threshold,
                 enable_output_image_publishing,
                 imgsz=(640, 640),  # inference size (height, width)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 classes=None,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 line_thickness=3,  # bounding box thickness (pixels)
                 half=False,  # use FP16 half-precision inference
                 dnn=False,  # use OpenCV DNN for ONNX inference
                 ):

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.line_thickness = line_thickness
        self.half = half
        self.dnn = dnn

        # get node name
        node_name = rospy.get_name()
        rospy.loginfo("{} node initializing".format(node_name))

        # ROS Topic names
        objects_array_topic = "{}/objects".format(node_name)
        output_image_topic = "{}/image".format(node_name)

        self.enable_output_image_publishing = enable_output_image_publishing

        # ROS publishers
        self.objects_array_pub = rospy.Publisher(
            objects_array_topic, ObjectsArray, queue_size=10)
        if self.enable_output_image_publishing:
            self.image_pub = rospy.Publisher(
                output_image_topic, Image, queue_size=1)

        # ROS subscribers
        self.image_sub = rospy.Subscriber(
            input_image_topic, Image, self.callback, queue_size=1)

        # get paths
        self.weights_pkg_path = rospkg.RosPack().get_path(weights_pkg_path)
        self.weights_path = os.path.join(
            self.weights_pkg_path, "net", "best.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "net", "config.yaml")
        self.confidence_threshold = confidence_threshold

        # init cv_bridge
        self.bridge = CvBridge()
        rospy.loginfo("before with torch.no_grad()")

        with torch.no_grad():
            rospy.loginfo("with torch.no_grad()")

            # Load model
            self.device = select_device(device)
            self.model = DetectMultiBackend(
                self.weights_path, device=self.device, dnn=dnn, data=self.config_path, fp16=half)
            self.stride, names, pt = self.model.stride, self.model.names, self.model.pt
            self.imgsz = check_img_size(
                self.imgsz, s=self.stride)  # check image size

            # Dataloader
            bs = 1  # batch_size

            # Run inference
            self.model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
            self.dt = [0.0, 0.0, 0.0]

    def callback(self, data):
        try:
            # convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # detect our objects
            objects, drawed_image = self.detector(cv_image)
            rospy.loginfo(objects)
            cv2.imshow("debug", drawed_image)

            # publish objects
            # init msg
            # self.object_msg = Object()
            # self.objects_array_msg = ObjectsArray()
            # for index, dnn_object in enumerate(dnn_objects):
            #     self.object_msg.name = dnn_object["name"].encode('utf-8')
            #     self.object_msg.confidence = dnn_object["confidence"]
            #     top_left_x = int(dnn_object['box'][0])
            #     self.object_msg.top_left_x = top_left_x
            #     top_left_y = int(dnn_object['box'][1])
            #     self.object_msg.top_left_y = top_left_y
            #     bottom_right_x = int(dnn_object['box'][2])
            #     self.object_msg.bottom_right_x = bottom_right_x
            #     bottom_right_y = int(dnn_object['box'][3])
            #     self.object_msg.bottom_right_y = bottom_right_y
            #     self.objects_array_msg.objects.append(self.object_msg)
            # self.objects_array_pub.publish(self.objects_array_msg)

            # if self.enable_output_image_publishing:
            #     # draw bounding boxes
            #     dnn_cv_image = self.draw(cv_image, dnn_objects)
            #     # convert cv image into ros format
            #     ros_image = self.bridge.cv2_to_imgmsg(dnn_cv_image, "bgr8")
            #     # publish output image
            #     self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def detector(self, img):
        with torch.no_grad():
            # Padded resize
            im = letterbox(img, new_shape=self.imgsz, stride=self.stride)[0]

            # Convert
            im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            im = np.ascontiguousarray(im)

            t1 = time_sync()
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            t2 = time_sync()
            self.dt[0] += t2 - t1

            # Inference
            pred = self.model(im, augment=self.augment, visualize=False)
            t3 = time_sync()
            self.dt[1] += t3 - t2

            # NMS
            pred = non_max_suppression(
                pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
            self.dt[2] += time_sync() - t3

            # Second-stage classifier (optional)
            # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
            # objects = self.deleteMultipleObjects(pred)

            # Process predictions
            for i, det in enumerate(pred):  # per image
                im0 = self.img.copy()

                annotator = Annotator(
                    im0, line_width=self.line_thickness, example=str(self.names))
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(
                        im.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        print(xyxy, conf, cls)
                        c = int(cls)  # integer class
                        label = f'{self.names[c]} {conf:.2f}'
                        annotator.box_label(xyxy, label, color=colors(c, True))

            # Stream results
            return pred, annotator.result()

    def deleteMultipleObjects(self, objects):
        """This function gets from labels.json a maximum number of objects that can be found and deletes unnecessary.

        Groups by name and kick out ones with lower probability
        """

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


if __name__ == '__main__':
    rospy.init_node('yolo_detector')
    # parameters
    input_image_topic = rospy.get_param('~input_image_topic')
    confidence_threshold = rospy.get_param('~dnn_confidence_threshold')
    enable_output_image_publishing = rospy.get_param(
        '~enable_output_image_publishing')
    weights_pkg_path = rospy.get_param('~dnn_weights_pkg')
    resize_input_to = rospy.get_param('~resize_input_to')
    try:
        ot = YoloDetector(weights_pkg_path, input_image_topic, confidence_threshold,
                          enable_output_image_publishing)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down {} node".format(rospy.get_name()))
