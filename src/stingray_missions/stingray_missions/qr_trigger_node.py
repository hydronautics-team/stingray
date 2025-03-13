from stingray_interfaces.srv import SetTransition
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TransitionTriggerNode(Node):

    def __init__(self):
        super().__init__('qr_trigger_node')

        self.declare_parameter(
            'transition_srv', '/stingray/services/transition')
        self.declare_parameter('zbar_topic', '/stingray/topics/zbar')

        self.transition_client = self.create_client(
            SetTransition,
            self.get_parameter(
                'transition_srv').get_parameter_value().string_value
        )
        if not self.transition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available...')

        self.get_logger().info('Service available, waiting for QR-code')
        self.subscription = self.create_subscription(
            String,
            self.get_parameter(
                'zbar_topic').get_parameter_value().string_value,
            self.qr_callback,
            1
        )
        self.saved_transition = None
        self.last_sent_time = 0.0

    def qr_callback(self, msg: String):
        current_time = time.time()
        # Если код совпадает с предыдущим, проверяем задержку 3 секунды
        if self.saved_transition == msg.data:
            if current_time - self.last_sent_time < 3:
                self.get_logger().info("Получен тот же QR-код, ждем 3 секунды перед повторной отправкой")
                return

        # Обновляем сохранённый код и время последней отправки
        self.saved_transition = msg.data
        self.last_sent_time = current_time

        self.get_logger().info("Получен код: " + msg.data)
        self.send_request()

    def send_request(self):
        self.future = self.transition_client.call_async(
            SetTransition.Request(transition=self.saved_transition)
        )


def main():
    rclpy.init()
    node = TransitionTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
