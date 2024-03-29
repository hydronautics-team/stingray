import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import torch
from functools import partial

from stingray_interfaces.msg import Bbox, BboxArray
from stingray_interfaces.srv import SetEnableObjectDetection
from stingray_object_detection.distance import DistanceCalculator


class YoloDetector(Node):
    def __init__(self,
                 node_name: str = 'yolov_detector',
                 detector_init_func: callable = None,
                 detector_inference_func: callable = None,
                 ):

        super().__init__(node_name)

        self.declare_parameter(
            'weights_pkg_name', 'stingray_object_detection')
        self.declare_parameter(
            'image_topic_list', ['/stingray/topics/front_camera'])
        self.declare_parameter(
            'debug', True)
        self.declare_parameter(
            'set_enable_object_detection_srv', '/stingray/services/set_enable_object_detection')

        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 20)

        # get weights path
        self.weights_pkg_path = f'{get_package_share_directory(self.get_parameter("weights_pkg_name").get_parameter_value().string_value)}'
        self.weights_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov8.pt")
        self.config_path = os.path.join(
            self.weights_pkg_path, "weights", "yolov8.yaml")

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

        self.set_enable_object_detection_service = self.create_service(
            SetEnableObjectDetection, self.get_parameter('set_enable_object_detection_srv').get_parameter_value().string_value, self._set_enable_object_detection)

        self.detection_enabled: dict[str, bool] = {}
        self.camera_info_subscriptions: dict[str, Subscription] = {}
        self.camera_info: dict[str, CameraInfo] = {}
        self.bbox_array_publishers: dict[str, Publisher] = {}
        self.image_publishers: dict[str, Publisher] = {}
        self.inited: dict[str, bool] = {}

        # init cv_bridge
        self.bridge = CvBridge()

        detector_init_func()
        self.detector = detector_inference_func

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
                self._image_callback, topic=input_topic)
            self.camera_info_subscriptions[input_topic] = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                camera_info_callback,
                1,
            )

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

    def _init_after_camera_info(self):
        """Initialize after camera info is available

        Returns:
            bool: True if inited, False otherwise
        """

        self.dist_calc = DistanceCalculator(
            imgsz=self.imgsz,
            fov=self.fov,
            object_attrs={
                'gate': [1.5, 1.5, 4/5, 'blue'],
                'yellow_flare': [0.15*2, 1.5, 1.5/5, 'yellow'],
                'red_flare': [0.15*2, 1.5, 1.5/5, 'red'],
                'blue_bowl': [0.7, 0.35, 0.8/5, 'blue'],
                'red_bowl': [0.7, 0.35, 0.8/5, 'red']
            }
        )

        return self.initialized

    def _set_enable_object_detection(self, request: SetEnableObjectDetection.Request, response: SetEnableObjectDetection.Response):
        """Callback to enable or disable object detection for specific camera topic

        Args:
            request (SetEnableObjectDetectionRequest): camera id and bool arg

        Returns:
            SetEnableObjectDetectionResponse: response with str message and success bool arg
        """

        self.detection_enabled[request.camera_topic] = request.enable
        response.success = True
        return response

    def detector(self, img):
        """ YOLO inference

        Args:
            img (_type_): cv2 image

        Returns:
            BboxArray: ros msg array with bboxes
            cv2 image: image with visualized bboxes
        """
        with torch.no_grad():
            # Padded resize
            letterbox = LetterBox(new_shape=self.imgsz, auto=False, stride=32)
            img = letterbox(image=img)
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
                    im0 = img.copy()
                    annotator = Annotator(
                        im0, line_width=self.line_thickness, example=str(self.names))
                for box in det.boxes:
                    # self.get_logger().info(f"xyxy: {xyxy}, label: {label}")
                    xyxy, label_id, confidence = box.xyxy[0].cpu(
                    ).detach().numpy(), box.cls.cpu(), box.conf.cpu()
                    label = self.names[int(label_id)]

                    if self.debug:
                        annotator.box_label(
                            xyxy, label, color=colors(int(label_id), True))

                    left, top, right, bottom = xyxy
                    distance, angle = self.dist_calc.calcDistanceAndAngle(
                        xyxy, label)

                    bbox_msg = Bbox()
                    bbox_msg.name = label
                    bbox_msg.confidence = float(confidence)
                    bbox_msg.top_left_x = int(left)
                    bbox_msg.top_left_y = int(top)
                    bbox_msg.bottom_right_x = int(right)
                    bbox_msg.bottom_right_y = int(bottom)
                    bbox_msg.distance = float(distance)
                    bbox_msg.angle = float(angle)

                    bbox_array_msg.bboxes.append(bbox_msg)

            return bbox_array_msg, annotator.result()

    def _camera_info_callback(self, info: CameraInfo, topic: str):
        self.camera_info[topic] = info
        self.destroy_subscription(self.camera_info_subscriptions[topic])
        del self.camera_info_subscriptions[topic]
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
            # try:
            # convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(input_image, "bgr8")

            # detect our objects
            bbox_array_msg, drawed_image = self.detector(cv_image)

            # publish results
            self.bbox_array_publishers[topic].publish(bbox_array_msg)
            if self.debug:
                ros_image = self.bridge.cv2_to_imgmsg(drawed_image, "bgr8")
                # publish output image
                self.image_publishers[topic].publish(ros_image)

            # except CvBridgeError as e:
            #     self.get_logger().error(f'CV Bridge error: {e}')
            # except Exception as e:
            #     self.get_logger().error(f'Error: {e}')


def main():
    rclpy.init(args=None)

    detector = YoloDetector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
