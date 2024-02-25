from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.logging import get_logger
from std_msgs.msg import String


class SubscriptionEvent:
    def __init__(self, transition_fn, topic: str, data: str, trigger: str, count: int = 0):
        """Event class for subscribing to a topic and triggering a transition when a certain message is received"""
        self._transition_fn = transition_fn
        self.topic = topic
        self.data = data
        self.trigger = trigger
        self.count = count

        self.subscription: Subscription = None

        self._counter = 0

    def subscribe(self, node: Node):
        """Subscribing to the topic"""
        raise NotImplementedError

    def unsubscribe(self, node: Node):
        """Unsubscribing from the topic"""
        if self.subscription:
            node.destroy_subscription(self.subscription)
            get_logger('event').info(f"Unsubscribed from {self.topic}")


class StringEvent(SubscriptionEvent):
    def __init__(self, transition_fn, topic: str, data: str, trigger: str, count: int = 0):
        super().__init__(transition_fn, topic, data, trigger, count)

    def subscribe(self, node: Node):
        """Subscribing to the topic"""
        self.subscription = node.create_subscription(
            String, self.topic, self._msg_callback, 10
        )
        get_logger('event').info(f"Subscribed to {self.topic}")

    def _msg_callback(self, msg: String):
        """Callback function for the topic subscription"""
        get_logger('event').info(f"Message received: {msg.data}")
        if msg.data == self.data:
            get_logger('event').info(f"Message matches: {self.data}")
            self._counter += 1
            if self._counter >= self.count:
                get_logger('event').info(f"Message count reached: {self.count}. Triggering: {self.trigger}")
                self._counter = 0
                self._transition_fn(self.trigger)
        else:
            self._counter = 0

class ObjectDetectionEvent(SubscriptionEvent):
    def __init__(self, transition_fn, topic: str, data: str, trigger: str, count: int = 0):
        super().__init__(transition_fn, topic, data, trigger, count)

    def subscribe(self, node: Node):
        """Subscribing to the topic"""
        self.subscription = node.create_subscription(
            String, self.topic, self._msg_callback, 10
        )
        get_logger('event').info(f"Subscribed to {self.topic}")

    # TODO
    def _msg_callback(self, msg: String):
        """Callback function for the topic subscription"""
        get_logger('event').info(f"Message received: {msg.data}")
        if msg.data == self.data:
            get_logger('event').info(f"Message matches: {self.data}")
            self._counter += 1
            if self._counter >= self.count:
                get_logger('event').info(f"Message count reached: {self.count}. Triggering: {self.trigger}")
                self._counter = 0
                self._transition_fn(self.trigger)
        else:
            self._counter = 0