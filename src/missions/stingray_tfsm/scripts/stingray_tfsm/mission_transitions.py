from abc import ABC, abstractmethod
from stingray_tfsm.pure_transitions import FSM_Simple
from stingray_tfsm.event import TopicEvent
import rospy

"""
Contains an abstract class for creating missions. It should unify the ROS state machine and required events.
Inspired by TimeEscaper
"""


class MissionBase(ABC):
    OTHER_DEFAULTS = None
    STATES = list()
    TRANSITIONS = list()

    @abstractmethod
    def setup_events(self, topic_name, object_name):
        """
        The setup_events function sets up the events for a specific topic and object.
        It is called by the setup_events function in the Topic class, which sets up events for all topics and objects.

        :param self: Access the attributes and methods of the class in python
        :param topic_name: Specify the name of the topic that will be used for communication
        :param object_name: Pass the name of the object that is being observed
        :return: self.events: boolean value that is True if all events are created successfully
        :doc-author: Trelent
        """
        self.specific_event = TopicEvent(topic_name, object_name)
        self.events = True
        return self.events

    @staticmethod
    def event_handler(event):
        """
        The event_handler function is a function that is called when the event is triggered.
        It returns True or False depending on whether the event was triggered or not.

        :param event: Pass the event that is being handled
        :return: True if the event is triggered and false if it is not
        :doc-author: Trelent
        """
        event.start_listening()
        rospy.sleep(0.5)
        value = event.is_triggered()
        event.stop_listening()
        return value

    @abstractmethod
    def reset(self, *args, **kwargs):
        """
        The reset function is called at the beginning of each trial. It is used to
        set the initial state of any variables that are needed for your condition
        script, and can also be used to reset ROS nodes between trials.

        :param self: Access the class's attributes and methods
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass a variable number of keyword arguments to a function
        :return: The scene dictionary
        :doc-author: Trelent
        """
        self.events = False
        self.setup_events(self.ros_parameter, self.OTHER_DEFAULTS)

        self.scene = {'condition_state': {
            'event_handler': MissionBase.event_handler,
            'args': self.specific_event
            }
        }

        self.machine = FSM_Simple(self.STATES, self.TRANSITIONS, self.scene)

    @abstractmethod
    def __init__(self, *args, **kwargs):
        """
        The __init__ function is called automatically whenever a new instance of the class is created.
        The __init__ function receives an instance of the class as its first parameter, followed by any number
        of parameters that are passed to the constructor. The __init__ function is used to initialize attributes
        of an object when it's created.

        :param self: Refer to the object instance itself
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass a keyworded, variable-length argument list
        :return: The object
        :doc-author: Trelent
        """
        self.ros_parameter = rospy.get_param('/control_fsm'+'/parameter_name', self.OTHER_DEFAULTS)

        self.specific_event = None

        self.scene = dict()
        self.machine = None

        self.events = False
        self.reset()

    @abstractmethod
    def verbose(self, verbose):
        """
        The verbose function is used to set machine's extended output

        :param self: Reference the class itself
        :param verbose: Determine whether to print the progress of the algorithm
        :return: None
        :doc-author: Trelent
        """
        self.machine.set_verbose(verbose)

    @abstractmethod
    def run(self, *args, **kwargs):
        """
        The run function is a relay to state machine's run function.

        :param self: Reference the current instance of the class
        :return: True if machine finished its tasks successfully
        :doc-author: Trelent
        """
        if self.machine is not None:
            return self.machine.run()
        else:
            raise TypeError("FSM was not initialized")



