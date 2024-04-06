from pathlib import Path
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


import numpy as np
import torch
from functools import partial

from stingray_interfaces.msg import Bbox, BboxArray
from stingray_interfaces.msg import EnableObjectDetection
from stingray_object_detection.distance import DistanceCalculator


class YoloDetectorBase(Node):
    def __init__(self,
                 node_name: str = 'yolo_detector',
                 ):
        
        """ Detecting objects on image

        Args:
            weights_pkg_name (str): name of ros package where to find weights
            image_topic_list (list): list of ROS topics with input images
            debug (bool): draw bboxes and publish image (for debugging)
            conf_thres (float, optional): confidence threshold. Defaults to 0.25.
            iou_thres (float, optional): NMS IOU threshold. Defaults to 0.45.
            max_det (int, optional): maximum detections per image. Defaults to 1000.
            device (str, optional): cuda device, i.e. 0 or 0,1,2,3 or cpu. Defaults to ''.
            tracker_max_age (int, optional): lifetime of tracked object. Defaults to 20.
            tracker_min_hits (int, optional): hits to start track object. Defaults to 20.
            tracker_iou_threshold (float, optional): IOU threshold for SORT-tracker. Defaults to 0.3.
        """

        super().__init__(node_name)

        self.declare_parameter(
            'weights_pkg_name', 'stingray_object_detection')
        self.declare_parameter(
            'bbox_attrs_pkg_name', 'stingray_object_detection')
        self.declare_parameter(
            'image_topic_list', ['/stingray/topics/front_camera'])
        self.declare_parameter(
            'debug', True)
        self.declare_parameter(
            'enable_object_detection_topic', '/stingray/topics/enable_object_detection')

        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 20)

        # get weights path
        bbox_attrs_pkg_path = f'{get_package_share_directory(self.get_parameter("bbox_attrs_pkg_name").get_parameter_value().string_value)}'
        bbox_attrs_config_path = Path(
            bbox_attrs_pkg_path) / "weights/bbox_attrs.yaml"
        with open(bbox_attrs_config_path, 'r') as f:
            bbox_attrs = yaml.load(f, Loader=Loader)
        self.dist_calc = DistanceCalculator(
            object_attrs=bbox_attrs
        )

        image_topic_list = self.get_parameter(
            'image_topic_list').get_parameter_value().string_array_value
        self.get_logger().info(f"image_topic_list: {image_topic_list}")

        self.debug = self.get_parameter(
            'debug').get_parameter_value().bool_value

        self.imgsz = None
        self.conf_thres = self.get_parameter(
            'confidence_threshold').get_parameter_value().double_value
        self.iou_thres = self.get_parameter(
            'iou_threshold').get_parameter_value().double_value
        self.max_det = self.get_parameter(
            'max_detections').get_parameter_value().integer_value
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")

        self._enable_object_detection_sub = self.create_subscription(
            EnableObjectDetection, 
            self.get_parameter('enable_object_detection_topic').get_parameter_value().string_value, 
            self._enable_object_detection,
            10)

        self.detection_enabled: dict[str, bool] = {}
        self.camera_info_subscriptions: dict[str, Subscription] = {}
        self.camera_info: dict[str, CameraInfo] = {}
        self.bbox_array_publishers: dict[str, Publisher] = {}
        self.image_publishers: dict[str, Publisher] = {}
        self.inited: dict[str, bool] = {}

        # init cv_bridge
        self.bridge = CvBridge()


        self.dt = [0.0, 0.0, 0.0]

        for input_topic in image_topic_list:

            # disable detection by default
            self.detection_enabled[input_topic] = False

            # ROS Topic names
            bbox_array_topic = f"{input_topic}/bbox_array"
            self.get_logger().info(
                f"input topic: {input_topic}, output bbox_array topic: {bbox_array_topic}")

            camera_info_topic = f"{input_topic}/camera_info"

            # ROS subscribers

            # provide topic name to callback
            input_img_callback = partial(
                self._image_callback, topic=input_topic)
            self.create_subscription(
                Image,
                input_topic,
                input_img_callback,
                1,
            )

            # provide topic name to callback
            camera_info_callback = partial(
                self._camera_info_callback, topic=input_topic)
            self.camera_info_subscriptions[input_topic] = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                camera_info_callback,
                1,
            )
            self.inited[input_topic] = False

            # ROS publishers
            bbox_array_pub = self.create_publisher(
                BboxArray, bbox_array_topic, 10)
            self.bbox_array_publishers[input_topic] = bbox_array_pub

            if self.debug:
                output_image_topic = f"{input_topic}/debug_image"
                self.get_logger().info("input topic: {}, output image topic: {}".format(
                    input_topic, output_image_topic))
                image_pub = self.create_publisher(
                    Image, output_image_topic, 1)
                self.image_publishers[input_topic] = image_pub

    def init_yolo(self, topic: str):
        """ YOLO init"""
        raise NotImplementedError

    def _enable_object_detection(self, msg: EnableObjectDetection):
        """Callback to enable or disable object detection for specific camera topic"""

        if msg.camera_topic == 'all':
            for key in self.detection_enabled.keys():
                self.detection_enabled[key] = msg.enable
                self.get_logger().info(f'Detection enabled: {self.detection_enabled}')
        else:
            self.detection_enabled[msg.camera_topic] = msg.enable
        self.get_logger().info(f'Detection enabled: {self.detection_enabled}')

    def detect(self, img: np.ndarray, topic: str):
        """ YOLO inference"""
        raise NotImplementedError

    def _camera_info_callback(self, camera_info: CameraInfo, topic: str):
        self.camera_info[topic] = camera_info
        self.destroy_subscription(self.camera_info_subscriptions[topic])
        del self.camera_info_subscriptions[topic]
        self.init_yolo(topic)
        self.inited[topic] = True

    def _image_callback(self, input_image: Image, topic: str):
        """ Input image callback

        Args:
            input_image (Image): ros image
            topic (str): topic name
        """
        if not self.detection_enabled[topic]:
            return
        if self.inited[topic]:
            try:
                # convert ROS image to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(input_image, "bgr8")

                # detect our objects
                bbox_array_msg, drawed_image = self.detect(cv_image, topic)

                # publish results
                self.bbox_array_publishers[topic].publish(bbox_array_msg)
                if self.debug:
                    ros_image = self.bridge.cv2_to_imgmsg(drawed_image, "bgr8")
                    # publish output image
                    self.image_publishers[topic].publish(ros_image)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error: {e}')
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
