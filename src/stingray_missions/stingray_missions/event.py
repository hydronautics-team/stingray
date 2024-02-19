import logging

from rclpy.node import Node
from std_msgs.msg import String


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TopicEvent:
    def __init__(self, trigger_fn, topic: str, data: str, trigger: str, count: int = 0):
        """Event class for subscribing to a topic and triggering a transition when a certain message is received"""
        self.trigger_fn = trigger_fn
        self.topic = topic
        self.data = data
        self.trigger = trigger
        self.count = count

        self.subsctiption = None

        self._counter = 0

    async def subscribe(self, node: Node):
        """Subscribing to the topic"""
        self.subsctiption = node.create_subscription(
            String, self.topic, self.msg_callback, 10
        )
        logger.info(f"Subscribed to {self.topic}")

    async def unsubscribe(self, node: Node):
        """Unsubscribing from the topic"""
        node.destroy_subscription(self.subsctiption)
        logger.info(f"Unsubscribed from {self.topic}")

    async def msg_callback(self, msg: String):
        """Callback function for the topic subscription"""
        logger.info(f"Message received: {msg.data}")
        if msg.data == self.data:
            logger.info(f"Message matches {self.data}")
            self._counter += 1
            if self._counter == self.count:
                logger.info(f"Message count reached {self.count}")
                self._counter = 0
                await self.trigger_fn(self.trigger)
        else:
            self._counter = 0
