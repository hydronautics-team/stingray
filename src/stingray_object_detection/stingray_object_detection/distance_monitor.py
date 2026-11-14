import math
import time

import rclpy
from rclpy.node import Node

from stingray_interfaces.msg import BboxArray


class DetectionDistanceMonitor(Node):
    """Log object ranges and closing rates from the common Bbox interface."""

    def __init__(self):
        super().__init__('detection_distance_monitor')
        self.declare_parameter(
            'bbox_topic', '/stingray/topics/camera/front/bbox_array')
        self.declare_parameter('log_interval', 1.0)
        self.declare_parameter('source', 'estimated')
        self._last_log = 0.0
        self._previous = {}
        topic = self.get_parameter('bbox_topic').value
        self.create_subscription(BboxArray, topic, self._callback, 10)
        self.get_logger().info(
            f"Monitoring detection distances on {topic} "
            f"(source={self.get_parameter('source').value})")

    def _callback(self, message):
        now = time.monotonic()
        interval = float(self.get_parameter('log_interval').value)
        if now - self._last_log < interval:
            return
        self._last_log = now

        observations = []
        for bbox in message.bboxes:
            coordinates = (bbox.pos_x, bbox.pos_y, bbox.pos_z)
            if not all(math.isfinite(value) for value in coordinates):
                observations.append(
                    f"{bbox.name}: distance=unknown conf={bbox.confidence:.2f}")
                continue

            distance = math.sqrt(sum(value * value for value in coordinates))
            rate_text = ''
            previous = self._previous.get(bbox.name)
            if previous is not None:
                elapsed = now - previous[0]
                if elapsed > 0.0:
                    closing_rate = (previous[1] - distance) / elapsed
                    rate_text = f" closing={closing_rate:+.2f}m/s"
            self._previous[bbox.name] = (now, distance)
            observations.append(
                f"{bbox.name}: forward={bbox.pos_z:.2f}m "
                f"distance={distance:.2f}m{rate_text} "
                f"conf={bbox.confidence:.2f}")

        if observations:
            source = self.get_parameter('source').value
            self.get_logger().info(
                f"Detections [{source}] | " + ' | '.join(observations))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionDistanceMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
