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
        self.declare_parameter(
            'zbar_topic', '/stingray/topics/zbar')

        self.transition_client = self.create_client(SetTransition, self.get_parameter(
            'transition_srv').get_parameter_value().string_value)
        while not self.transition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service avaible, waiting qr-code')
        self.subscription = self.create_subscription(
            String,
            self.get_parameter('zbar_topic').get_parameter_value().string_value,
            self.qr_callback,
            10)
        self.saved_transition = None
        self.msg_repeated = 0

    def qr_callback(self, msg: String):
        self.get_logger().info("Got code: " + msg.data)
        if (self.saved_transition == msg.data):
            self.msg_repeated += 1
            if (self.msg_repeated > 5):
                service_responce = self.send_request()
                if (service_responce):
                    self.get_logger().info("Launch successful")
                else:
                    self.get_logger().error("Launch unsuccesful")
                self.msg_repeated = 0
        else:
            self.msg_repeated = 0
            self.saved_transition = msg.data

    def send_request(self):
        self.future = self.transition_client.call_async(SetTransition.Request(transition=self.saved_transition))
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().ok
    
    def run(self):
        while rclpy.ok():
            service_responce = self.send_request(self.saved_transition)
            if (service_responce):
                self.get_logger().info("Launch successful")
            else:
                self.get_logger().error("Launch unsuccesful")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1e-1)


def main():
    rclpy.init()

    node = TransitionTriggerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
