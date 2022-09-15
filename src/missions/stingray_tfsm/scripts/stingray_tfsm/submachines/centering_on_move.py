from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectOnRight, ObjectOnLeft


class CenteringOnMoveSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 camera: str,
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 6,
                 angle: int = 15):
        """ Submission for centering on object in camera

        Args:
            name (str): mission name
            camera (str): camera name
            target (str): object name
            confirmation (int, optional): confirmation value of continuously detected object
             after which will be event triggered. Defaults to 2.
            tolerance (int, optional): centering tolerance. Defaults to 14.
        """
        self.name = '_'+name
        self.d_angle = angle
        self.target = target
        self.confirmation = confirmation
        self.tolerance = tolerance
        self.camera = camera

        self.gate_detected = None
        self.gate_lefter = None
        self.gate_righter = None
        super().__init__(name)

    def setup_states(self):
        states = ('condition_detected',
                  'condition_lefter', 'condition_righter',
                  'move_rotate_clock', 'move_rotate_anti',
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [self.machine.state_init, 'move_rotate_clock' + self.name,
                                             'move_rotate_anti' + self.name], 'condition_detected' + self.name],

            ['condition_f', 'condition_detected' + self.name, self.machine.state_aborted],
            ['condition_s', 'condition_detected' + self.name, 'condition_lefter' + self.name],

            ['condition_f', 'condition_lefter' + self.name, 'condition_righter' + self.name],
            ['condition_s', 'condition_lefter' + self.name, 'move_rotate_anti' + self.name],

            ['condition_f', 'condition_righter' + self.name, 'condition_lefter' + self.name],
            ['condition_s', 'condition_righter' + self.name, 'move_rotate_clock' + self.name],
        ]

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.machine.auv.execute_move_goal,
                "args": ({
                    'march': 0.5,
                    'lag': 0.0,
                    'yaw': 0,
                },),
            },
            'condition_detected' + self.name: {
                'condition': self.event_handler,
                'args': (self.gate_detected,)
            },
            'condition_righter' + self.name: {
                'condition': self.event_handler,
                'args': (self.gate_righter,)
            },
            'condition_lefter' + self.name: {
                'condition': self.event_handler,
                'args': (self.gate_lefter,)
            },
            'move_rotate_anti' + self.name: {
                'march': 0.7,
                'lag': 0.0,
                'yaw': -self.d_angle,
            },
            'move_rotate_clock' + self.name: {
                'march': 0.7,
                'lag': 0.0,
                'yaw': self.d_angle,
            },
            self.machine.state_aborted: {
                'preps': self.machine.auv.execute_stop_goal,
                "args": (),
            },
        }

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation)
        self.gate_lefter = ObjectOnLeft(
            get_objects_topic(self.camera), self.target, self.confirmation, tolerance=self.tolerance * 0.01)
        self.gate_righter = ObjectOnRight(
            get_objects_topic(self.camera), self.target, self.confirmation, tolerance=self.tolerance * 0.01)

    def check_machine(self):
        if type(self.machine) is AUVStateMachine or \
                type(self.machine) is PureStateMachine:
            return 1
        else:
            print(type(self.machine))
            raise TypeError("machine was not initialized")

    def set_init_state(self,):
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
