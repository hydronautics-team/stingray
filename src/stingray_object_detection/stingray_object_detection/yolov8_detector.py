import rclpy
from ament_index_python import get_package_share_directory

import numpy as np
import os
import torch

from stingray_object_detection.yolo_detector_base import YoloDetectorBase
from stingray_interfaces.msg import Bbox, BboxArray

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
            input_img (numpy.ndarray): cv2 image

        Returns:
            BboxArray: ros msg array с финальными детекциями
            cv2 image: исходное изображение с нарисованными детекциями
        """
        with torch.no_grad():
            # Padded resize
            letterbox = LetterBox(auto=True, stride=32)
            img = letterbox(image=input_img)
            # Преобразуем из HWC в CHW, BGR -> RGB
            im = img.transpose((2, 0, 1))[::-1]
            im = np.ascontiguousarray(im)

            t1 = time_sync()
            im = torch.from_numpy(im).to(self.device)
            im = im.half()
            im /= 255.0  # приводим пиксели от 0..255 к 0..1

            if len(im.shape) == 3:
                im = im[None]  # добавляем batch dim

            t2 = time_sync()
            self.dt[0] += t2 - t1

            # Inference
            pred = self.model.predict(im)
            t3 = time_sync()
            self.dt[1] += t3 - t2

            # Подготовим результирующий msg с детекциями
            bbox_array_msg = BboxArray()

            for det in pred:
                # Если debug=True, то будем рисовать
                if self.debug:
                    im0 = input_img.copy()
                    annotator = Annotator(
                        im0, line_width=3, example=str(self.names))

                # Словарь для хранения лучшей детекции по каждому классу
                best_detections = {}

                # Проходимся по всем детекциям: оставляем только одну с max confidence
                for box in det.boxes:
                    xyxy, label_id, confidence = (
                        box.xyxy[0].cpu().detach().numpy(),
                        int(box.cls.cpu()),
                        float(box.conf.cpu())
                    )
                    label = self.names[label_id]

                    # Масштабируем координаты к размеру исходного input_img
                    xyxy = scale_boxes(
                        im.shape[2:], xyxy, input_img.shape).round()

                    # Смотрим, есть ли уже детекция данного класса
                    if label not in best_detections or confidence > best_detections[label]['confidence']:
                        # Считаем дистанцию и углы для текущей детекции
                        pos_x, pos_y, pos_z, horiz_angle, vert_angle = self.dist_calc.calcDistanceAndAngle(
                            xyxy, label, self.camera_info[topic]
                        )
                        best_detections[label] = {
                            'confidence': confidence,
                            'xyxy': xyxy,
                            'label_id': label_id,
                            'pos_x': pos_x,
                            'pos_y': pos_y,
                            'pos_z': pos_z,
                            'horiz_angle': horiz_angle,
                            'vert_angle': vert_angle
                        }

                # Теперь формируем выходные bbox'ы только из best_detections
                for label, det_info in best_detections.items():
                    c = det_info['confidence']
                    xyxy = det_info['xyxy']
                    lbl_id = det_info['label_id']

                    # При debug рисуем бокс и надпись
                    if self.debug:
                        annotator.box_label(
                            xyxy,
                            label,
                            color=colors(lbl_id, True)
                        )

                    # Создаём ros-сообщение
                    bbox_msg = Bbox()
                    bbox_msg.name = label
                    bbox_msg.confidence = float(c)
                    bbox_msg.top_left_x = int(xyxy[0])
                    bbox_msg.top_left_y = int(xyxy[1])
                    bbox_msg.bottom_right_x = int(xyxy[2])
                    bbox_msg.bottom_right_y = int(xyxy[3])
                    bbox_msg.pos_x = float(det_info['pos_x'])
                    bbox_msg.pos_y = float(det_info['pos_y'])
                    bbox_msg.pos_z = float(det_info['pos_z'])
                    bbox_msg.horizontal_angle = float(det_info['horiz_angle'])
                    bbox_msg.vertical_angle = float(det_info['vert_angle'])

                    bbox_array_msg.bboxes.append(bbox_msg)

            # Если debug=True, то возвращаем изображение с нарисованными боксами
            if self.debug:
                return bbox_array_msg, annotator.result()
            else:
                # Иначе возвращаем оригинальное
                return bbox_array_msg, input_img


def main():
    rclpy.init(args=None)

    detector = YoloV8Detector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
