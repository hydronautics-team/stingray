import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class AsyncSubscription:

    def __init__(self, node: Node, type, topic: str, qos):
        self.message_queue = asyncio.Queue(maxsize=qos)
        node.create_subscription(type, topic, self.msg_callback, qos)

    async def msg_callback(self, msg):
        await self.message_queue.put(msg)

    async def messages(self):
        while True:
            yield await self.message_queue.get()


class AsyncActionClient(ActionClient):
    client_goal_handle = None

    def cancel(self):
        if self.client_goal_handle:
            self.client_goal_handle.cancel_goal_async()

    async def send_goal_async(self, goal_msg):
        _send_goal_future = super().send_goal_async(
            goal_msg,
        )
        self.client_goal_handle = await asyncio.ensure_future(_send_goal_future)
        if not self.client_goal_handle.accepted:
            raise Exception("Goal rejected.")
        _get_result_future = self.client_goal_handle.get_result_async()
        result_goal_handle = await asyncio.ensure_future(_get_result_future)
        return result_goal_handle