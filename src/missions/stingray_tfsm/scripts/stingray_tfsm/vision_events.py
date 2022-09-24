from stingray_tfsm.core.pure_events import TopicEvent
from stingray_object_detection_msgs.msg import ObjectsArray


DEFAULT_RANGE_W = 800
DEFAULT_RANGE_H = 800
DEFAULT_TOLERANCE = 0.15
DEFAULT_CONFIDENCE = 0.65

# distance to height for yellow flare: 180 for 3, 90 for 6


def width(_obj):
    return abs(_obj.top_left_x - _obj.bottom_right_x)  #


def height(_obj):
    return abs(_obj.top_left_y - _obj.bottom_right_y)


def calculate_proximity(tlx, brx, tly, bry, mrange):
    proximity = abs(tlx - brx)
    proximity += abs(tly - bry)
    proximity = 1 - mrange / proximity * 0.43
    # print("Ya sotvoril dich: ", proximity)

    return proximity


def calculate_center_x(_obj):
    return (_obj.top_left_x + _obj.bottom_right_x) // 2


def calculate_center_y(_obj):
    return (_obj.top_left_y + _obj.bottom_right_y) // 2


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
            pos = calculate_center_x(obj)
            c = pos - memorized
            direction = 1 if c >= 0 else -1
            c = abs(c)
            obj_to_asses.append((c, direction, pos))

    result = min(obj_to_asses)
    return result[0]*result[1], result[2]


def is_big(_obj, border_h=DEFAULT_RANGE_H):
    if height(_obj) / border_h > 0.5:
        return True
    else:
        return False


# todo unite all this mess into one class in order to check events analyzing only one message for all cases
class ObjectDetectionEvent(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 1, queue_size: int = 10, tolerance=DEFAULT_TOLERANCE, confidence=DEFAULT_CONFIDENCE):
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
        self._target_object = object_name
        self._confidence = confidence
        self._tolerance = tolerance

        self.current_center_x = None
        self.relative_shift_x = 0
        self.current_center_y = None
        self.relative_shift_y = 0

        self.current_width = None
        self.current_height = None

        self.current_object = None
        if self._target_object == 'red_bowl' or\
                self._target_object == 'yellow_flare' or\
                self._target_object == 'blue_bowl':
            self._confidence -= 0.3
    
    def is_big(self):
        if height(self.current_object) / 480 > 0.5:
            return True
        else:
            return False

    def get_track(self):
        return self.current_center_x

    def get_better_track_x(self):
        return self.current_center_x, self.relative_shift_x

    def get_better_track_y(self):
        return self.current_center_y, self.relative_shift_y

    def get_x_offset(self):
        return self.current_center_x - DEFAULT_RANGE_W // 2

    def get_y_offset(self):
        return self.current_center_y - DEFAULT_RANGE_H // 2

    def righter(self):
        if self.get_x_offset() / DEFAULT_RANGE_W >= 0:
            return 1
        else:
            return 0

    def lefter(self):
        if self.get_x_offset() / DEFAULT_RANGE_W <= - 0:
            return 1
        else:
            return 0

    def higher(self):
        if self.get_y_offset() / DEFAULT_RANGE_H * 100 >= self._tolerance:
            return 1
        else:
            return 0

    def lower(self):
        if self.get_y_offset() / DEFAULT_RANGE_H * 100 <= -self._tolerance:
            return 1
        else:
            return 0

    def is_close(self):
        pass

    def is_ortho(self):
        if 1 - self._tolerance <= self.current_width/self.current_height <= 1 + self._tolerance:
            print(f"{self._target_object} is orthogonal to AUV")
            return 1
        else:
            print(f"{self._target_object} is NOT orthogonal to AUV")
            return 0

    def _trigger_fn(self, msg: ObjectsArray):
        _obj = get_best_object(msg.objects, self._target_object, self._confidence)
        if _obj:
            self.current_object = _obj
            if self.current_center_x is not None:
                self.relative_shift_x, self.current_center_x =\
                    get_closest_to_memorized(msg.objects, self._target_object,
                                             self.current_center_x + self.relative_shift_x)
            else:
                self.relative_shift_x, self.current_center_x = 0, calculate_center_x(_obj)
            self.current_center_y = calculate_center_y(_obj)
            self.current_width = width(_obj)
            self.current_height = height(_obj)
            return 1
        return 0


class ObjectIsCloseEvent(TopicEvent):
    """An event that is triggered when specific object is detected in object detection topic.
    """

    def __init__(self, topic_name: str, object_name: str,
                 n_triggers: int = 2, queue_size=None, _range=DEFAULT_RANGE_W,
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
                 n_triggers: int = 1, queue_size=None, _range=DEFAULT_RANGE_W,
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
                 n_triggers: int = 1, queue_size=None, _range=DEFAULT_RANGE_W,
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

