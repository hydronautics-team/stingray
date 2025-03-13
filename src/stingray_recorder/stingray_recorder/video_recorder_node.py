#!/usr/bin/env python3

import cv2
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def opencv_version():
    v = cv2.__version__.split('.')[0]
    if v == '2':
        return 2
    elif v == '3':
        return 3
    elif v == '4':
        return 4
    raise Exception('opencv version can not be parsed. v={}'.format(v))


class VideoRecorderNode(Node):
    def __init__(self, name):
        super().__init__(name)
        # Объявление параметров
        self.declare_parameter('source_topic', '/stingray/topics/camera')
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        self.declare_parameter('output_fps', 15)
        self.declare_parameter('output_format', 'h264')
        self.declare_parameter('record_dir', "./records/")
        self.declare_parameter('set_recording_srv',
                               '/stingray/services/set_recording_srv')

        # Получение параметров
        self.source_topic = self.get_parameter(
            'source_topic').get_parameter_value().string_value
        self.output_width = self.get_parameter(
            'output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter(
            'output_height').get_parameter_value().integer_value
        self.output_fps = self.get_parameter(
            'output_fps').get_parameter_value().integer_value
        self.output_format = self.get_parameter(
            'output_format').get_parameter_value().string_value
        self.record_dir = self.get_parameter(
            'record_dir').get_parameter_value().string_value
        self.set_recording_srv = self.get_parameter(
            'set_recording_srv').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Подписка на топик изображений
        self.image_sub = self.create_subscription(
            Image, self.source_topic, self.callback_image, 10)

        # Сервисы для старта и остановки записи
        self.enable_recording_service = self.create_service(SetBool, self.get_parameter(
            'set_recording_srv').get_parameter_value().string_value, self.enable_recording)

        self.recording = False
        self.video_writer = None

        self.get_logger().info("Video Recorder Node инициализирована.")

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Ошибка конвертации изображения: {e}')
            return

        if self.recording and self.video_writer is not None:
            self.video_writer.write(cv_image)

    def enable_recording(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            return self.start_recording(response)
        else:
            return self.stop_recording(response)

    def start_recording(self, response: SetBool.Response):
        if self.recording:
            response.success = False
            response.message = "Запись уже запущена."
            return response

        # Формирование пути для сохранения: record_dir/YYYY_MM_DD/topic_name/
        date_str = datetime.datetime.now().strftime("%Y_%m_%d")
        topic_name = self.source_topic.lstrip('/').replace('/', '_')
        base_path = Path(self.record_dir) / date_str / topic_name
        base_path.mkdir(parents=True, exist_ok=True)

        # Формирование имени файла с отметкой времени
        timestamp_str = datetime.datetime.now().strftime("%H_%M_%S")
        filename = f"{timestamp_str}.avi"
        full_path = str(base_path / filename)

        # Инициализация VideoWriter
        opencv_ver = opencv_version()
        if opencv_ver == 2:
            fourcc = cv2.cv.FOURCC(*self.output_format)
        elif opencv_ver in [3, 4]:
            fourcc = cv2.VideoWriter_fourcc(*self.output_format)
        else:
            self.get_logger().error("Неподдерживаемая версия OpenCV.")
            response.success = False
            response.message = "Неподдерживаемая версия OpenCV."
            return response

        self.video_writer = cv2.VideoWriter(full_path, fourcc, self.output_fps,
                                            (self.output_width, self.output_height))
        if not self.video_writer.isOpened():
            response.success = False
            response.message = f"Не удалось открыть видеозапись для файла {full_path}."
            self.get_logger().error(response.message)
            return response

        self.recording = True
        response.success = True
        response.message = f"Запись начата. Файл: {full_path}"
        self.get_logger().info(response.message)
        return response

    def stop_recording(self, response: SetBool.Response):
        if not self.recording:
            response.success = False
            response.message = "Запись не активна."
            return response

        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        self.recording = False
        response.success = True
        response.message = "Запись остановлена."
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode("stingray_video_recorder")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Прерывание с клавиатуры. Завершение работы.")
    finally:
        if node.video_writer is not None:
            node.video_writer.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
