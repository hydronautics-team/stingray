#!/usr/bin/env python3

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from cv_bridge import CvBridge, CvBridgeError
from stingray_interfaces.msg import Bbox, BboxArray
from stingray_interfaces.srv import SetEnableObjectDetection
from stingray_object_detection.yolo_detector_base import YoloDetectorBase
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python import get_package_share_directory
import os
import sys
import torch
from functools import partial

import numpy as np
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import check_img_size, non_max_suppression, scale_boxes
from yolov5.utils.plots import Annotator, colors
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.augmentations import letterbox


class YoloV5Detector(YoloDetectorBase):
    def __init__(self):
        """ YOLO v5"""
        super().__init__('yolov5_detector')

        # get weights path
        self.weights_pkg_path = f'{get_package_share_directory(self.get_parameter("weights_pkg_name").get_parameter_value().string_value)}'
        self.weights_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov5.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov5.yaml")

    def init_yolo(self, topic: str):
        """ YOLO init"""
        with torch.no_grad():
            # Load model
            self.device = select_device('')
            self.model = DetectMultiBackend(
                self.weights_path, device=self.device, data=self.config_path)
            self.stride, self.names, pt = self.model.stride, self.model.names, self.model.pt

            imgsz = (self.camera_info[topic].height, self.camera_info[topic].width)
            self.imgsz = check_img_size(
                imgsz, s=self.stride)  # check image size

            # Dataloader
            bs = 1  # batch_size

            # Run inference
            self.model.warmup(
                imgsz=(1 if pt else bs, 3, *self.imgsz))  # warmup

    def detect(self, img: np.ndarray, topic: str):
        """ YOLO inference

        Args:
            img (_type_): cv2 image

        Returns:
            BboxArray: ros msg array with bboxes
            cv2 image: image with visualized bboxes
        """
        with torch.no_grad():
            # Padded resize
            im = letterbox(img, new_shape=(640,640), stride=self.stride)[0]
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
                pred, self.conf_thres, self.iou_thres, max_det=self.max_det)
            self.dt[2] += time_sync() - t3

            # Second-stage classifier (optional)
            # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
            # objects = self.deleteMultipleObjects(pred)

            # Process predictions
            bbox_array_msg = BboxArray()

            for det in pred:  # per image
                if self.debug:
                    im0 = img.copy()
                    annotator = Annotator(
                        im0, line_width=3, example=str(self.names))
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(
                        im.shape[2:], det[:, :4], img.shape).round()

                    # for cpu and gpu machines
                    det = det.cpu().detach().numpy()
                    dots = np.asarray(det).reshape(-1, 6)

                    # update tracking objects
                    # dots_tracked = self.tracker.update(dots)

                    # for *xyxy, c, id in reversed(dots_tracked):
                    #     label = self.names[int(c)]
                    #     if self.debug:
                    #         annotator.box_label([xyxy[0], xyxy[1], xyxy[2], xyxy[3]], label + ' ' + str(int(id)),
                    #                             color=colors(int(c), True))

                    #     object_msg = Bbox()
                    #     object_msg.id = int(id)
                    #     object_msg.name = label
                    #     object_msg.top_left_x = int(xyxy[0])
                    #     object_msg.top_left_y = int(xyxy[1])
                    #     object_msg.bottom_right_x = int(xyxy[2])
                    #     object_msg.bottom_right_y = int(xyxy[3])
                    #     objects_array_msg.bboxes.append(object_msg)

                    for *xyxy, score, label_id in reversed(dots):
                        label = self.names[int(label_id)]
                        # self.get_logger().info(
                        #     f"xyxy: {xyxy}, score: {score}, label: {label}")
                        if self.debug:
                            annotator.box_label([xyxy[0], xyxy[1], xyxy[2], xyxy[3]], label,
                                                color=colors(int(label_id), True))
                            
                        pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = self.dist_calc.calcDistanceAndAngle(xyxy, label, self.camera_info[topic])

                        bbox_msg = Bbox()
                        bbox_msg.confidence = float(score)
                        bbox_msg.name = label
                        bbox_msg.top_left_x = int(xyxy[0])
                        bbox_msg.top_left_y = int(xyxy[1])
                        bbox_msg.bottom_right_x = int(xyxy[2])
                        bbox_msg.bottom_right_y = int(xyxy[3])
                        bbox_msg.pos_x = float(pos_x)
                        bbox_msg.pos_y = float(pos_y)
                        bbox_msg.pos_z = float(pos_z)
                        bbox_msg.horizontal_angle = float(horizontal_angle)
                        bbox_msg.vertical_angle = float(vertical_angle)

                        bbox_array_msg.bboxes.append(bbox_msg)

            # Stream results
            return bbox_array_msg, annotator.result()


def main():
    rclpy.init(args=None)

    detector = YoloV5Detector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
