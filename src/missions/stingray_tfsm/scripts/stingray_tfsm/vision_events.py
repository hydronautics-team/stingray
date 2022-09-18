from stingray_tfsm.core.pure_events import TopicEvent
from stingray_object_detection_msgs.msg import ObjectsArray


DEFAULT_RANGE = 640
DEFAULT_TOLERANCE = 0.15
DEFAULT_CONFIDENCE = 0.65

# distance to height for yellow flare: 180 for 3, 90 for 6


def width(_obj):
    return abs(_obj.top_left_x - _obj.bottom_right_x)


def height(_obj):
    return abs(_obj.top_left_y - _obj.bottom_right_y)


def calculate_proximity(tlx, brx, tly, bry, mrange):
    proximity = abs(tlx - brx)
    proximity += abs(tly - bry)
    proximity = 1 - mrange / proximity * 0.43
    # print("Ya sotvoril dich: ", proximity)

    return proximity


def calculate_center(_obj):
    return (_obj.top_left_x + _obj.bottom_right_x) // 2


def very_close(tlx, brx, tly, bry, mrange, target, *args, **kwargs):
    prox = calculate_proximity(tlx, brx, tly, bry, mrange)
    if target == 'red_flare':
        prox += 0.45
    if prox < 0:
        print(f'too far away to assess {target}')
        return False
    print(prox)
    if target == 'gate':
        close = (1 - (1 - prox) * 100) * 100
    else:
        close = prox*2
    print(close, end='\n-====-\n')
    return close > 0.80


def get_best_object(objects, target_name, req_confidence):
    obj_to_num = list(enumerate([obj.name for obj in objects]))
    ii = -1
    ii_conf = -1
    if obj_to_num:
        for i, name in obj_to_num:
            if target_name == name:
                if objects[i].confidence >= req_confidence and objects[i].confidence >= ii_conf:
                    ii = i
                    ii_conf = objects[i].confidence
        if ii > -1:
            _obj = objects[ii]
        else:
            _obj = None
        return _obj
    else:
        return None


def get_closest_to_memorized(objects, target_name, memorized):
    obj_to_asses = []
    for obj in objects:
        if obj.name == target_name:
            pos = calculate_center(obj)
            c = pos - memorized
            direction = 1 if c >= 0 else -1
            c = abs(c)
            obj_to_asses.append((c, direction, pos))

    result = min(obj_to_asses)
    return result[0]*result[1], result[2]


# todo unite all this mess into one class in order to check events analyzing only one message for all cases
class ObjectDetectionEvent(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size=None, confidence=DEFAULT_CONFIDENCE):
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
        self._confidence = confidence
        self.current_center = None
        self.relative_shift = 0
        self.current_object = None
    
    def is_big(self):
        if height(self.current_object) / 480 > 0.5:
            return True
        else:
            return False

    def get_track(self):
        return self.current_center

    def _trigger_fn(self, msg: ObjectsArray):
        _obj = get_best_object(msg.objects, self._object_name, self._confidence)
        if _obj:
            self.current_object = _obj
            if self.current_center is not None:
                self.relative_shift, self.current_center =\
                    get_closest_to_memorized(msg.objects, self._object_name, self.current_center + self.relative_shift)
            else:
                self.relative_shift, self.current_center = 0,  calculate_center(_obj)
            return 1
        return 0


class ObjectIsCloseEvent(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 2, queue_size=None, _range=DEFAULT_RANGE,
                 tolerance=DEFAULT_TOLERANCE, confidence=DEFAULT_CONFIDENCE):
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
        self._range = _range
        self.center = _range / 2
        self._tolerance = tolerance
        self._confidence = confidence
        if object_name == 'yellow_flare':
            self._confidence -= 0.3

    def _trigger_fn(self, msg: ObjectsArray):
        _obj = get_best_object(msg.objects, self._object_name, self._confidence)
        if not _obj:
            return 0
        return very_close(
            _obj.top_left_x, _obj.bottom_right_x,
            _obj.top_left_y, _obj.bottom_right_y,
            self._range, self._object_name
        )


class ObjectOnRight(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size=None, _range=DEFAULT_RANGE,
                 tolerance=DEFAULT_TOLERANCE, confidence=DEFAULT_CONFIDENCE):
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
        self._range = _range
        self.center = _range / 2
        self._tolerance = tolerance
        self._confidence = confidence
        if object_name == 'yellow_flare':
            self._confidence -= 0.3

    def _trigger_fn(self, msg: ObjectsArray):
        _obj = get_best_object(msg.objects, self._object_name, self._confidence)
        if not _obj:
            return 0

        proximity_allowance = calculate_proximity(
            _obj.top_left_x, _obj.bottom_right_x,
            _obj.top_left_y, _obj.bottom_right_y,
            self._range
        )
        center = (_obj.top_left_x + _obj.bottom_right_x) // 2
        proximity_allowance = max(proximity_allowance, self._tolerance)
        if self._range // 2 - center >= 0 and \
                abs(1 - center / self.center) >= proximity_allowance:
            return 1
        return 0


class ObjectOnLeft(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size=None, _range=DEFAULT_RANGE,
                 tolerance=DEFAULT_TOLERANCE, confidence=DEFAULT_CONFIDENCE):
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
        self._range = _range
        self.center = _range / 2
        self._tolerance = tolerance
        self._confidence = confidence
        if object_name == 'yellow_flare':
            self._confidence -= 0.3

    def _trigger_fn(self, msg: ObjectsArray):
        _obj = get_best_object(msg.objects, self._object_name, self._confidence)
        if not _obj:
            return 0
        proximity_allowance = calculate_proximity(
            _obj.top_left_x, _obj.bottom_right_x,
            _obj.top_left_y, _obj.bottom_right_y,
            self._range
        )
        center = (_obj.top_left_x + _obj.bottom_right_x) // 2
        proximity_allowance = max(proximity_allowance, self._tolerance)
        if self._range // 2 - center <= 0 and \
                abs(1 - center / self.center) >= proximity_allowance:
            return 1
        else:
            return 0


class ObjectOrtho(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size=None, _range=DEFAULT_RANGE,
                 tolerance=DEFAULT_TOLERANCE, confidence=DEFAULT_CONFIDENCE):
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
        self._range = _range
        self._tolerance = tolerance
        self._confidence = confidence
        if object_name == 'yellow_flare':
            self._confidence -= 0.3

    def _trigger_fn(self, msg: ObjectsArray):
        obj_to_num = list(enumerate([obj.name for obj in msg.objects]))
        for i, name in obj_to_num:
            if self._object_name == name:
                if msg.objects[i].confidence >= self._confidence:
                    _obj = msg.objects[i]
                    x_side = _obj.top_left_x + _obj.bottom_right_x
                    y_side = _obj.top_left_y + _obj.bottom_right_y
                    print()
                    if x_side / y_side > self._tolerance:
                        return 1
        return 0
