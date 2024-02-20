import sys
from stingray_interfaces.srv import TransitionSrv
from stingray_utils.config import StingrayConfig

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionLibraryClient(Node):

    def __init__(self):
        super().__init__('mission_library_client')
        self.cli = self.create_client(TransitionSrv, StingrayConfig.ros.services["stingray_missions"]["transition"])
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service avaible, waiting qr-code')
        self.req = TransitionSrv.Request()
        self.subscription = self.create_subscription(
            String,
            'barcode',
            self.qr_callback,
            10)
        self.subscription
        self.got_msg = False
        self.command = ""

    def qr_callback(self, msg):
        self.get_logger().info("Got qr-code: "+msg.data)
        self.got_msg = True
        self.command = msg.data

    def send_request(self, com):
        self.got_msg = False
        self.req.transition = com
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    ml_client = MissionLibraryClient()

    while rclpy.ok():
        if(ml_client.got_msg):
            service_responce = ml_client.send_request(ml_client.command).ok
            if(service_responce):
                ml_client.get_logger().info("Launch successful")
            else:
                ml_client.get_logger().error("Launch unsuccesful")
        else:
            rclpy.spin_once(ml_client, timeout_sec=0.1)

    ml_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()