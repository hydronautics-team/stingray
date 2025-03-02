import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import torch
from functools import partial

from stingray_object_detection.yolo_detector_base import YoloDetectorBase
from stingray_interfaces.msg import Bbox, BboxArray
from stingray_interfaces.msg import EnableObjectDetection
from stingray_object_detection.distance import DistanceCalculator

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from ultralytics.data.augment import LetterBox
from ultralytics.utils.torch_utils import select_device, time_sync
from ultralytics.utils.ops import scale_boxes


class YoloV8Detector(YoloDetectorBase):
    def __init__(self):
        """ YOLO v8"""
        super().__init__('yolov8_detector')

    def init_yolo(self, topic: str):
        # get weights path
        self.weights_pkg_path = f'{get_package_share_directory(self.get_parameter("weights_pkg_name").get_parameter_value().string_value)}'
        self.weights_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov8.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov8.yaml")

        with torch.no_grad():
            # Load model
            self.device = select_device(self.device)
            self.model = YOLO(model=self.weights_path)
            self.names = self.model.names
            self.get_logger().info(
                f'Model inited from {self.weights_path} for topic {topic}')

    def detect(self, input_img: np.ndarray, topic: str):
        """ YOLO inference

        Args:
            img (_type_): cv2 image

        Returns:
            BboxArray: ros msg array with bboxes
            cv2 image: image with visualized bboxes
        """
        with torch.no_grad():
            # imgsz = (self.camera_info[topic].height, self.camera_info[topic].width)
            # Padded resize
            letterbox = LetterBox(auto=True, stride=32)
            img = letterbox(image=input_img)
            # Convert
            im = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            im = np.ascontiguousarray(im)

            t1 = time_sync()
            im = torch.from_numpy(im).to(self.device)
            # im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im = im.half()
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            t2 = time_sync()
            self.dt[0] += t2 - t1

            # Inference
            pred = self.model.predict(im)
            t3 = time_sync()
            self.dt[1] += t3 - t2

            # Process predictions
            bbox_array_msg = BboxArray()

            for det in pred:  # per image
                if self.debug:
                    im0 = input_img.copy()
                    annotator = Annotator(
                        im0, line_width=3, example=str(self.names))

                # Rescale boxes from img_size to im0 size
                # self.get_logger().info(f"im.shape: {im.shape}")
                # self.get_logger().info(f"input_img.shape: {input_img.shape}")
                # self.get_logger().info(f"det: {det}")
                # self.get_logger().info(f"det.boxes: {det.boxes}")
                for box in det.boxes:
                    xyxy, label_id, confidence = box.xyxy[0].cpu(
                    ).detach().numpy(), box.cls.cpu(), box.conf.cpu()
                    # self.get_logger().info(f"xyxy: {xyxy}")
                    label = self.names[int(label_id)]

                    # self.get_logger().info(f"prev xyxy: {xyxy}")
                    # xyxy = scale_boxes(
                    #     im.shape[2:], xyxy, input_img.shape).round()
                    # self.get_logger().info(f"xyxy: {xyxy}")

                    # left, top, right, bottom = xyxy

                    # top = 0 if top < 12 else top - 12
                    # bottom = 360 if bottom > 371 else bottom - 24

                    # scale = self.camera_info[topic].width / img.shape[1]

                    # new_xyxy = [left * scale, top * scale, right * scale, bottom * scale]

                    if self.debug:
                        annotator.box_label(
                            xyxy, label, color=colors(int(label_id), True))

                    pos_x, pos_y, pos_z, horizontal_angle, vertical_angle = self.dist_calc.calcDistanceAndAngle(
                        xyxy, label, self.camera_info[topic])

                    bbox_msg = Bbox()
                    bbox_msg.name = label
                    bbox_msg.confidence = float(confidence)
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

            return bbox_array_msg, annotator.result()


def main():
    rclpy.init(args=None)

    detector = YoloV8Detector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
