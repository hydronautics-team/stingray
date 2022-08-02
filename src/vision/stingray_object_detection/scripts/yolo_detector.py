#!/usr/bin/env python3

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from stingray_object_detection_msgs.msg import Object, ObjectsArray
from stingray_object_detection_msgs.srv import SetEnableObjectDetection, SetEnableObjectDetectionResponse, SetEnableObjectDetectionRequest
from sensor_msgs.msg import Image
import os
import sys
import torch
import numpy as np
import json

sys.path.insert(1, os.path.join(rospkg.RosPack().get_path(
    "stingray_object_detection"), "scripts/yolov5"))
from utils.augmentations import letterbox
from utils.torch_utils import select_device, time_sync
from utils.plots import Annotator, colors
from utils.general import (
    check_img_size, non_max_suppression, scale_coords)
from models.common import DetectMultiBackend

class YoloDetector:
    def __init__(self,
                 weights_pkg_name,
                 image_topic_list,
                 debug=False,
                 imgsz=(640, 640),
                 conf_thres=0.25,
                 iou_thres=0.45,
                 max_det=1000,
                 device='',
                 classes=None,
                 agnostic_nms=False,
                 line_thickness=3):
        """ Detecting objects on image

        Args:
            weights_pkg_name (str): name of ros package where to find weights
            image_topic_list (list): list of ROS topics with input images
            enable_output_image_publishing (bool): draw bboxes and publish image (for debugging)
            imgsz (tuple, optional): inference size (height, width). Defaults to (640, 640).
            conf_thres (float, optional): confidence threshold. Defaults to 0.25.
            iou_thres (float, optional): NMS IOU threshold. Defaults to 0.45.
            max_det (int, optional): maximum detections per image. Defaults to 1000.
            device (str, optional): cuda device, i.e. 0 or 0,1,2,3 or cpu. Defaults to ''.
            classes (_type_, optional): filter by class: --class 0, or --class 0 2 3. Defaults to None.
            agnostic_nms (bool, optional): class-agnostic NMS. Defaults to False.
            line_thickness (int, optional): bounding box thickness (pixels). Defaults to 3.
        """
        # get weights path
        self.weights_pkg_path = rospkg.RosPack().get_path(weights_pkg_name)
        self.weights_path = os.path.join(
            self.weights_pkg_path, "weights", "best.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "weights", "config.yaml")

        self.image_topic_list = image_topic_list.split(" ")
        self.debug = debug

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.line_thickness = line_thickness

        # configs
        stingray_resources_path = rospkg.RosPack().get_path("stingray_resources")
        with open(os.path.join(stingray_resources_path, "configs/ros.json")) as f:
            self.ros_config = json.load(f)

        # get node name
        node_name = rospy.get_name()
        rospy.loginfo("{} node initializing".format(node_name))

        set_enable_object_detection_service = rospy.Service(
            self.ros_config['services']['set_enable_object_detection'], SetEnableObjectDetection, self.set_enable_object_detection)

        self.detection_enabled = {}
        self.objects_array_publishers = {}
        self.image_publishers = {}

        for input_topic in self.image_topic_list:
            # disable detection by default
            self.detection_enabled[input_topic] = False

            # ROS Topic names
            objects_array_topic = "%s%s/objects" % (input_topic, node_name)
            rospy.loginfo("Node: {}, input topic: {}, output objects topic: {}".format(
                node_name, input_topic, objects_array_topic))

            # ROS subscribers
            self.image_sub = rospy.Subscriber(
                input_topic, Image, self.image_callback, callback_args=input_topic, queue_size=1)

            # ROS publishers
            objects_array_pub = rospy.Publisher(
                objects_array_topic, ObjectsArray, queue_size=10)
            self.objects_array_publishers[input_topic] = objects_array_pub

            if self.debug:
                output_image_topic = "%s%s/image" % (input_topic, node_name)
                rospy.loginfo("Node: {}, input topic: {}, output image topic: {}".format(
                    node_name, input_topic, output_image_topic))
                image_pub = rospy.Publisher(
                    output_image_topic, Image, queue_size=1)
                self.image_publishers[input_topic] = image_pub

        # init cv_bridge
        self.bridge = CvBridge()

        with torch.no_grad():
            # Load model
            self.device = select_device(device)
            self.model = DetectMultiBackend(
                self.weights_path, device=self.device, data=self.config_path)
            self.stride, self.names, pt = self.model.stride, self.model.names, self.model.pt

            self.imgsz = check_img_size(
                self.imgsz, s=self.stride)  # check image size

            # Dataloader
            bs = 1  # batch_size

            # Run inference
            self.model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
            self.dt = [0.0, 0.0, 0.0]

            # to check if inited
            self.initialized = True

    def set_enable_object_detection(self, request: SetEnableObjectDetectionRequest) -> SetEnableObjectDetectionResponse:
        """Callback to enable or disable object detection for specific camera topic

        Args:
            request (SetEnableObjectDetectionRequest): camera id and bool arg

        Returns:
            SetEnableObjectDetectionResponse: response with str message and success bool arg
        """
        self.detection_enabled[self.image_topic_list[request.camera_id]
                               ] = request.enabled
        response = SetEnableObjectDetectionResponse()
        response.success = True
        return response

    def detector(self, img):
        """ YOLO inference

        Args:
            img (_type_): cv2 image

        Returns:
            ObjectsArray: ros msg array with bboxes
            cv2 image: image with visualized bboxes
        """
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
            pred = self.model(im)
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
            objects_array_msg = ObjectsArray()

            for i, det in enumerate(pred):  # per image
                im0 = img.copy()
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
                        c = int(cls)  # integer class
                        label = f'{self.names[c]} {conf:.2f}'

                        # draw bboxes if enabled
                        if self.debug:
                            annotator.box_label(
                                xyxy, label, color=colors(c, True))

                        # rospy.loginfo("conf type: {0}".format(type(float(conf.cpu().detach().numpy()))))
                        # rospy.loginfo("conf: {0}".format(conf.cpu().detach().numpy()))
                        # rospy.loginfo("xyxy[0] type: {0}".format(type(xyxy[0].cpu().detach().numpy())))
                        # rospy.loginfo("xyxy[0]: {0}".format(xyxy[0].cpu().detach().numpy()))

                        object_msg = Object()
                        object_msg.name = self.names[c]
                        object_msg.confidence = float(
                            conf.cpu().detach().numpy())
                        object_msg.top_left_x = int(
                            xyxy[0].cpu().detach().numpy())
                        object_msg.top_left_y = int(
                            xyxy[1].cpu().detach().numpy())
                        object_msg.bottom_right_x = int(
                            xyxy[2].cpu().detach().numpy())
                        object_msg.bottom_right_y = int(
                            xyxy[3].cpu().detach().numpy())
                        objects_array_msg.objects.append(object_msg)

            # Stream results
            return objects_array_msg, annotator.result()

    def image_callback(self, input_image: Image, topic: str):
        """ Input image callback

        Args:
            input_image (Image): ros image
            topic (str): topic name
        """
        if not self.detection_enabled[topic]:
            return
        if hasattr(self, 'initialized'):
            try:
                # convert ROS image to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(input_image, "bgr8")

                # detect our objects
                objects_array_msg, drawed_image = self.detector(cv_image)

                # publish results
                self.objects_array_publishers[topic].publish(objects_array_msg)
                if self.debug:
                    ros_image = self.bridge.cv2_to_imgmsg(drawed_image, "bgr8")
                    # publish output image
                    self.image_publishers[topic].publish(ros_image)

            except CvBridgeError as e:
                rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('yolo_detector')
    # parameters
    weights_pkg_name = rospy.get_param('~weights_pkg_name')
    image_topic_list = rospy.get_param('~image_topic_list')
    debug = rospy.get_param('~debug')
    try:
        ot = YoloDetector(weights_pkg_name, image_topic_list, debug)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down {} node".format(rospy.get_name()))
