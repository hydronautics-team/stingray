from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectOnRight, ObjectOnLeft
from rospy import loginfo

class CenteringAngleSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 camera: str,
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 6,
                 angle: int = 5):
        """ Submission for centering on object in camera

        Args:
            name (str): mission name
            camera (str): camera name
            target (str): object name
            confirmation (int, optional): confirmation value of continuously detected object after which will be event triggered. Defaults to 2.
            tolerance (int, optional): centering tolerance. Defaults to 14.
        """
        self.name = '_' + name
        self.d_angle = angle
        self.target = target
        self.confirmation = confirmation
        self.tolerance = tolerance
        self.camera = camera
        self.previous = 0  # or 1
        self.give_up = 0
        self.give_up_threshold = 5

        self.gate_detected = None
        self.gate_lefter = None
        self.gate_righter = None
        super().__init__(name)

    def reset_freeze(self):
        self.give_up = 0

    def setup_states(self):
        states = ('condition_detected',
                  'condition_lefter', 'condition_righter',
                  'rotate_clock', 'rotate_anti',
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
                   [self.machine.transition_start, [self.machine.state_init, 'rotate_clock' + self.name,
                                                    'rotate_anti' + self.name], 'condition_detected' + self.name],

                   ['condition_f', 'condition_detected' + self.name, self.machine.state_aborted],
                   ['condition_s', 'condition_detected' + self.name, 'condition_lefter' + self.name],

                   ['condition_f', 'condition_lefter' + self.name, 'condition_righter' + self.name],
                   ['condition_s', 'condition_lefter' + self.name, 'rotate_anti' + self.name],

                   ['condition_f', 'condition_righter' + self.name, self.machine.state_end],
                   ['condition_s', 'condition_righter' + self.name, 'rotate_clock' + self.name],
               ]

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation)
        self.gate_lefter = ObjectOnLeft(
            get_objects_topic(self.camera), self.target, self.confirmation, tolerance=self.tolerance * 0.01)
        self.gate_righter = ObjectOnRight(
            get_objects_topic(self.camera), self.target, self.confirmation, tolerance=self.tolerance * 0.01)

    def conditioned_handler(self, event):
        value = self.event_handler(event)
        loginfo(f"\nDEBUG: current condition is {value}; event is {event}; previous is {self.previous} \n")
        if value == 1 and (event is self.gate_lefter and self.previous == 'righter' or
                           event is self.gate_righter and self.previous == 'lefter'):
            self.give_up += 1
            loginfo(f"DEBUG: Cannot center precisely. Did {self.give_up} wobbles")
        else:
            loginfo("DEBUG: Centering is going ok")

        if self.give_up > self.give_up_threshold:
            loginfo("DEBUG: Cannot center precisely. Consider the centering is done")
            value = 0
        if event is self.gate_righter:
            self.previous = 'righter'
        else:
            self.previous = 'lefter'
        return value

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.reset_freeze,
                'args': ()
            },
            'condition_detected' + self.name: {
                'condition': self.event_handler,
                'args': (self.gate_detected,)
            },
            'condition_righter' + self.name: {
                'condition': self.conditioned_handler,
                'args': (self.gate_righter,)
            },
            'condition_lefter' + self.name: {
                'condition': self.conditioned_handler,
                'args': (self.gate_lefter,)
            },
            'rotate_anti' + self.name: {
                'angle': -self.d_angle
            },
            'rotate_clock' + self.name: {
                'angle': self.d_angle
            }
        }

    def check_machine(self):
        if type(self.machine) is AUVStateMachine or \
                type(self.machine) is PureStateMachine:
            return 1
        else:
            print(type(self.machine))
            raise TypeError("machine was not initialized")

    def set_init_state(self, ):
        if self.check_machine():
            self.machine.set_state(self.machine.state_init)

    def set_state(self, state):
        if self.check_machine():
            self.machine.set_state(state)

    def run(self):
        if self.check_machine():
            value = self.machine.run()
            return value

    def verbose(self, verbose):
        if self.check_machine():
            self.machine.verbose(verbose)
