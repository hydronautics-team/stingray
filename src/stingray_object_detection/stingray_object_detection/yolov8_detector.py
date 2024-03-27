import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import torch
from functools import partial

from stingray_interfaces.msg import Bbox, BboxArray
from stingray_interfaces.srv import SetEnableObjectDetection
from stingray_object_detection.distance import DistanceCalculator

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from ultralytics.data.augment import LetterBox
from ultralytics.utils.torch_utils import select_device, time_sync


class YoloDetector(Node):
    def __init__(self,
                 imgsz=(480, 640),
                 conf_thres=0.25,
                 iou_thres=0.45,
                 max_det=1000,
                 device='',
                 classes=None,
                 agnostic_nms=False,
                 line_thickness=3,
                 fov=60,
                 ):

        super().__init__('yolov8_detector')

        self.declare_parameter(
            'weights_pkg_name', 'stingray_object_detection')
        self.declare_parameter(
            'image_topic_list', ['/stingray/topics/front_camera'])
        self.declare_parameter(
            'debug', True)
        self.declare_parameter(
            'set_enable_object_detection_srv', '/stingray/services/set_enable_object_detection')

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

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.line_thickness = line_thickness
        self.fov = fov

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

        self.set_enable_object_detection_service = self.create_service(
            SetEnableObjectDetection, self.get_parameter('set_enable_object_detection_srv').get_parameter_value().string_value, self._set_enable_object_detection)

        self.detection_enabled: dict[str, bool] = {}
        self.objects_array_publishers: dict[str, Publisher] = {}
        self.image_publishers: dict[str, Publisher] = {}

        # init cv_bridge
        self.bridge = CvBridge()

        with torch.no_grad():
            # Load model
            self.device = select_device(device)
            self.model = YOLO(model=self.weights_path)
            self.names = self.model.names
            
        self.dt = [0.0, 0.0, 0.0]
        # to check if inited
        self.initialized = True

        for input_topic in image_topic_list:

            # disable detection by default
            self.detection_enabled[input_topic] = True

            # ROS Topic names
            objects_array_topic = f"{input_topic}/objects"
            self.get_logger().info(
                f"input topic: {input_topic}, output objects topic: {objects_array_topic}")

            # provide topic name to callback
            bind = partial(self.image_callback, topic=input_topic)

            # ROS subscribers
            self.create_subscription(
                Image,
                input_topic,
                bind,
                1,
            )

            # ROS publishers
            objects_array_pub = self.create_publisher(
                BboxArray, objects_array_topic, 10)
            self.objects_array_publishers[input_topic] = objects_array_pub

            if self.debug:
                output_image_topic = f"{input_topic}/debug_image"
                self.get_logger().info("input topic: {}, output image topic: {}".format(
                    input_topic, output_image_topic))
                image_pub = self.create_publisher(
                    Image, output_image_topic, 1)
                self.image_publishers[input_topic] = image_pub

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
            objects_array_msg = BboxArray()

            for det in pred:  # per image
                if self.debug:
                    im0 = img.copy()
                    annotator = Annotator(
                        im0, line_width=self.line_thickness, example=str(self.names))
                for box in det.boxes:
                    # self.get_logger().info(f"xyxy: {xyxy}, label: {label}")
                    xyxy, label_id = box.xyxy[0].cpu(
                    ).detach().numpy(), box.cls
                    label = self.names[int(label_id)]

                    if self.debug:
                        annotator.box_label(
                            xyxy, label, color=colors(int(label_id), True))

                    left, top, right, bottom = xyxy

                    # self.get_logger().info(str(ImageHandler.calcDistanceAndAngle([obj], None)))
                    # objects.append(obj)
                    distance, angle = self.dist_calc.calcDistanceAndAngle(xyxy, label)

                    object_msg = Bbox()
                    object_msg.name = label
                    object_msg.top_left_x = int(left)
                    object_msg.top_left_y = int(top)
                    object_msg.bottom_right_x = int(right)
                    object_msg.bottom_right_y = int(bottom)
                    object_msg.distance = float(distance)
                    object_msg.angle = float(angle)

                    objects_array_msg.bboxes.append(object_msg)

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
            # try:
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
