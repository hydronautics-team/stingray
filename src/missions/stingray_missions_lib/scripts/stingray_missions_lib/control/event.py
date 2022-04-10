from abc import ABC, abstractmethod
import rospy
from object_detection_msgs.msg import ObjectsArray

"""@package docstring
Contains an abstract class for event base and implementations for some common events.
"""


class EventBase(ABC):
    """An abstract class for creating events.

    Child classes should implement event logic. When the event is happening, child class must set
    field _is_happened to True. It's up to child class whether to make event reusable or not.
    """

    @abstractmethod
    def start_listening(self):
        """Enables event listening.
        """
        pass

    @abstractmethod
    def stop_listening(self):
        """Disables event listening.
        """

    @abstractmethod
    def is_triggered(self):
        """Returns whether event has happened.

        :return: True if event has happened, False otherwise.
        """
        pass


class TopicEvent(EventBase):
    """An event that is triggered when specific message appears in topic for some time.
    """

    # TODO: Improve concurrency safety

    def __init__(self, topic_name: str, topic_type, trigger_fn,
                 n_triggers: int = 1, trigger_reset: bool = True, queue_size: int = 10):
        """The constructor.

        :param topic_name: Name of the topic.
        :param topic_type: Topic type (as data_class parameter in rospy.Subscriber).
        :param trigger_fn: A function with one argument - message that comes from topic. Function must return True
        if message is relevant, False otherwise.
        :param n_triggers: Number of sequential appearances of relevant messages (see trigger_fn param) in topic to count
        event as triggered.
        :param trigger_reset: If True, zeroes trigger counter after each appearance of non-relevant messages.
        :param queue_size: Queue size for topic (as queue_size parameter in rospy.Subscriber).
        """
        super().__init__()
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._trigger_fn = trigger_fn
        self._n_triggers = n_triggers
        self._trigger_reset = trigger_reset
        self._queue_size = queue_size
        self._is_triggered = False
        self._triggers_count = 0
        self._topic_sub: rospy.Subscriber = None

    def _topic_callback(self, msg):
        is_triggered = self._trigger_fn(msg)
        if not is_triggered:
            if self._trigger_reset:
                self._triggers_count = 0
            return
        self._triggers_count += 1
        if self._triggers_count >= self._n_triggers:
            self._is_triggered = True
            self._topic_sub.unregister()

    def start_listening(self):
        self._is_triggered = False
        self._triggers_count = 0
        self._topic_sub = rospy.Subscriber(self._topic_name,
                                           self._topic_type,
                                           callback=self._topic_callback,
                                           queue_size=self._queue_size)

    def stop_listening(self):
        self._topic_sub.unregister()

    def is_triggered(self):
        return self._is_triggered


class ObjectDetectionEvent(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size: int = 10):
        """The constructor.

        :param topic_name: Object detection topic name.
        :param object_name: Name of the object class of interest.
        :param n_triggers: Number of sequential detections to define object as detected. Used to cope with
        false-positive detections. Counter is zeroed after each non-detections.
        :param queue_size: Queue size for topic (as queue_size parameter in rospy.Subscriber).
        """

        super().__init__(topic_name=topic_name,
                         topic_type=ObjectsArray,
                         trigger_fn=self._trigger_fn,
                         n_triggers=n_triggers,
                         trigger_reset=True,
                         queue_size=queue_size)
        self._object_name = object_name

    def _trigger_fn(self, msg: ObjectsArray):
        return self._object_name in [obj.name for obj in msg.objects]
