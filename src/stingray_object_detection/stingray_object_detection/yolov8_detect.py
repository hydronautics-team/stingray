import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory

import os
import torch

from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator, colors
# from stingray_object_detection_msgs.msg import Object, ObjectsArray
from stingray_interfaces.msg import Bbox, BboxArray
from ultralytics.data.augment import LetterBox
from ultralytics.utils.torch_utils import select_device, time_sync
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class YoloDetector(Node):
    def __init__(self,
                 imgsz=(640, 640)):
        
        super().__init__('yolov8_detector')

        self.imgsz = imgsz
        
        self.declare_parameter(name="weights_pkg_name", value="stingray_object_detection")        
        self.declare_parameter(name="image_topic_list", value="/image_raw")
        self.declare_parameter('debug', True)
        # self.declare_parameter('set_enable_object_detection_srv', '/stingray/services/set_enable_object_detection')

        self.objects_array_publishers = {}
        self.image_publishers = {}
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.weights_pkg_path = self.get_parameter("weights_pkg_name").get_parameter_value().string_value
        self.weights_path = os.path.join(self.weights_pkg_path, "weights", "best.pt")
        self.config_path = os.path.join(self.weights_pkg_path, "weights", "config.yaml")
        
        image_topic_list_str = self.get_parameter(
            'image_topic_list').get_parameter_value().string_value
        self.image_topic_list = set(image_topic_list_str.strip('[]').split(' ')) 

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.model = YOLO(model=self.weights_path)

        for input_topic in self.image_topic_list:

            # ROS Topic names
            node_name = "stingray_object_detection"
            objects_array_topic = "objects_array_topic"
            output_image_topic = "output_image_topic"

            self.get_logger().info("Node: {}, input topic: {}, output objects topic: {}".format(
                node_name, input_topic, objects_array_topic))

            # ROS subscribers
            self.image_sub = self.create_subscription(
                Image,
                input_topic,
                lambda x: self.image_callback(x, input_topic),
                1,
            )

            # ROS publishers
            objects_array_pub = self.create_publisher(
                BboxArray, objects_array_topic, 10)
            self.objects_array_publishers[input_topic] = objects_array_pub

            if self.debug:
                self.get_logger().info("Node: {}, input topic: {}, output image topic: {}".format(
                    node_name, input_topic, output_image_topic))
                image_pub = self.create_publisher(
                    Image, output_image_topic, 1)
                self.image_publishers[input_topic] = image_pub

    def detector(self, cv_image):
        with torch.no_grad():
            # im = LetterBox(cv_image)
            # Convert
            im = cv_image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            im = np.ascontiguousarray(im)

            # t1 = time_sync()
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.model.__getattr__ else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0]
            # t2 = time_sync()
            # self.dt[0] += t2 - t1

            # self.get_logger().info("get new im !!!OMG!!!", im)
            results = self.model.predict(im)
            objects_array_msg = BboxArray()

            for r in results:
                
                # f.write("new im\n")
                # im0 = input_image.copy()

                annotator = Annotator(cv_image)
                # print("r = ", r)
                boxes = r.boxes
                # print("boxes = ", boxes)

                for box in boxes:
                    
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    label = self.model.names[int(c)]

                    if self.debug:
                        annotator.box_label(b, label, color=colors(int(c), True))

                    left, top, right, bottom = np.array(box.xyxy.cpu(), dtype=int).squeeze()
                    width = right - left
                    height = bottom - top
                    center = (left + int((right-left)/2), top + int((bottom-top)/2))
                    confidence = float(box.conf.cpu())
                    
                    object_msg = Bbox()
                    object_msg.name = label
                    object_msg.confidence = confidence
                    object_msg.top_left_x = int(left)
                    object_msg.top_left_y = int(top)
                    object_msg.bottom_right_x = int(right)
                    object_msg.bottom_right_y = int(bottom)

                    objects_array_msg.bboxes.append(object_msg)

            return objects_array_msg, annotator.result()



    def image_callback(self, input_image: Image, topic: str):
   
        # self.get_logger().info(topic)

        cv_image = self.bridge.imgmsg_to_cv2(input_image, "bgr8")

        objects_array_msg, drawed_image = self.detector(cv_image)

        self.objects_array_publishers[topic].publish(objects_array_msg)

        if self.debug:
            ros_image = self.bridge.cv2_to_imgmsg(drawed_image, "bgr8")
            self.image_publishers[topic].publish(ros_image)





def main():
    rclpy.init(args=None)

    node = rclpy.create_node('stingray_object_detection')
    node.get_logger().info('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

    # parameters
    # node.declare_parameter(name="weights_pkg_name", value="stingray_object_detection")
    # node.declare_parameter(name="image_topic_list", value="/image_raw")

    # weights_pkg_name = node.get_parameter('weights_pkg_name').get_parameter_value().string_value
    # image_topic_list = node.get_parameter('image_topic_list').get_parameter_value().string_value
    # debug = node.get_parameter('debug').get_parameter_value().string_value

    detector = YoloDetector()
    rclpy.spin(detector)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()