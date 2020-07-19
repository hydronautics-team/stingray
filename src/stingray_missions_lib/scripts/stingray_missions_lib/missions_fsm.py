import rospy
import smach
import actionlib

from abc import ABCMeta, abstractmethod
from threading import Lock


class Mission(smach.State):
    __metaclass__ = ABCMeta

    def __init__(self, mission_name):
        self.outcome_ok_ = mission_name.upper() + "_OK"
        self.outcome_failed_ = mission_name.upper() + "_FAILED"
        smach.State.__init__(self, outcomes=[self.outcome_ok_, self.outcome_failed_])

    @abstractmethod
    def execute(self, userdata):
        pass


class MissionsFSMFactory(smach.StateMachine):
    __metaclass__ = ABCMeta

    def __init__(self):
        self.outcome_ok_ = "MISSIONS_OK"
        self.outcome_failed_ = "MISSIONS_FAILED"

    @abstractmethod
    def create_fsm(self):
        pass


class Observer:
    __metaclass__ = ABCMeta

    def __init__(self, topic_name, topic_type):
        self.topic_name_ = topic_name
        self.topic_type_ = topic_type
        self.callback_ = None
        self.topic_count_ = None
        self.topic_count_threshold_ = None
        self.topic_count_reset_ = None
        self.subscriber_ = None

    @abstractmethod
    def topic_transform_(self, message):
        pass

    def topic_callback_(self, message):
        data = self.topic_transform_(message)
        if data is None:
            if self.topic_count_reset_:
                self.topic_count_ = 0
            return

        self.topic_count_ += 1
        if self.topic_count_ != self.topic_count_threshold_:
            return

        self.subscriber_.unregister()
        self.callback_(data)

    def set_callback(self, callback):
        self.callback_ = callback

    def start(self, threshold=1, reset=False):
        self.topic_count_ = 0
        self.topic_count_threshold_ = threshold
        self.topic_count_reset_ = reset
        self.subscriber_ = rospy.Subscriber(self.topic_name_, self.topic_type_, self.topic_callback_)

    def stop(self):
        self.subscriber_.unregister()


class ObserverActionExecutor:
    def __init__(self, action_name, action_type, observers):
        self.action_name_ = action_name
        self.action_type_ = action_type
        self.observers_ = observers
        self.ready_ = False
        self.outcome_ = None
        self.lock_ = Lock()
        self.action_client_ = actionlib.SimpleActionClient(self.action_name_, self.action_type_)

    def set_outcome_(self, outcome, cancel_goal=False):
        self.lock_.acquire()
        if not self.ready_:
            self.ready_ = True
            self.outcome_ = outcome
        if cancel_goal:
            self.action_client_.cancel_goal()
        self.lock_.release()

    def execute(self, action_goal):
        self.ready_ = False
        self.outcome_ = None
        self.action_client_.wait_for_server()

        for observer in self.observers_:
            observer[0].set_callback(lambda data, outcome=observer[1]: self.set_outcome_(outcome, True))
            observer[0].start()

        self.action_client_.send_goal(action_goal,
                                      active_cb=None,
                                      feedback_cb=lambda: self.set_outcome_(None),
                                      done_cb=None)
        while not self.ready_:
            pass

        for observer in self.observers_:
            observer[0].stop()
        return self.outcome_
