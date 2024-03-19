import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import Image

import os
import torch

from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator, colors
# from stingray_object_detection_msgs.msg import Object, ObjectsArray
from stingray_interfaces.msg import Object, ObjectsArray
from ultralytics.data.augment import LetterBox
from ultralytics.utils.torch_utils import select_device, time_sync
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class YoloDetector(Node):
    def __init__(self,
                 weights_pkg_name,
                 image_topic_list,
                 imgsz=(640, 640)):
        
        super().__init__('stingray_object_detection')

        self.imgsz = imgsz
        
        self.weights_pkg_path = weights_pkg_name
        self.objects_array_publishers = {}
        self.image_publishers = {}
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


        # self.get_logger().info(weights_pkg_name)
        self.weights_path = os.path.join(
            self.weights_pkg_path, "weights", "best.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "weights", "config.yaml")
        
        self.image_topic_list = image_topic_list.split(" ")
        
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
                ObjectsArray, objects_array_topic, 10)
            self.objects_array_publishers[input_topic] = objects_array_pub

            self.get_logger().info("Node: {}, input topic: {}, output image topic: {}".format(
                node_name, input_topic, output_image_topic))
            image_pub = self.create_publisher(
                Image, output_image_topic, 1)
            self.image_publishers[input_topic] = image_pub


    def image_callback(self, input_image: Image, topic: str):
   
            self.get_logger().info('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!next!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            # self.get_logger().info(topic)

            cv_image = self.bridge.imgmsg_to_cv2(input_image, "bgr8")

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
                objects_array_msg = ObjectsArray()

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

                        annotator.box_label(b, label, color=colors(int(c), True))
                        # print(self.model.names[int(c)], " = ", box.xyxy[0])


                        left, top, right, bottom = np.array(box.xyxy.cpu(), dtype=int).squeeze()
                        width = right - left
                        height = bottom - top
                        center = (left + int((right-left)/2), top + int((bottom-top)/2))
                        confidence = float(box.conf.cpu())
                        
                        object_msg = Object()
                        object_msg.name = label
                        object_msg.confidence = confidence
                        object_msg.top_left_x = int(left)
                        object_msg.top_left_y = int(top)
                        object_msg.bottom_right_x = int(right)
                        object_msg.bottom_right_y = int(bottom)

                        objects_array_msg.objects.append(object_msg)

                        print(object_msg)
                        
                self.objects_array_publishers[topic].publish(objects_array_msg)
                        
                results_im = annotator.result() 
                ros_image = self.bridge.cv2_to_imgmsg(results_im, "bgr8")
                self.image_publishers[topic].publish(ros_image)





def main():
    rclpy.init(args=None)

    node = rclpy.create_node('stingray_object_detection')
    node.get_logger().info('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

    # parameters
    node.declare_parameter(name="weights_pkg_name", value="stingray_object_detection")
    node.declare_parameter(name="image_topic_list", value="/image_raw")

    weights_pkg_name = node.get_parameter('weights_pkg_name').get_parameter_value().string_value
    image_topic_list = node.get_parameter('image_topic_list').get_parameter_value().string_value
    # debug = node.get_parameter('debug').get_parameter_value().string_value

    detector = YoloDetector(weights_pkg_name = weights_pkg_name, image_topic_list = image_topic_list)
    rclpy.spin(detector)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()