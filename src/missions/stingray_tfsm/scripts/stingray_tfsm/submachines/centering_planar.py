from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.vision_events import ObjectDetectionEvent
from stingray_object_detection.utils import get_objects_topic
from rospy import loginfo


class CenteringPlanarSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 camera: str,  # bottom needed
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 9,
                 angle: int = 8):
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
        self.wobbles = 0
        self.give_up_threshold = 15
        self.lag_speed
        self.march_speed

        self.gate_detected = None
        super().__init__(name)

    def reset_freeze(self):
        self.wobbles = 0

    def setup_states(self):
        states = ('condition_detected', 'condition_done'
                  'move_adjust',
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [self.machine.state_init], 'condition_detected' + self.name],

            ['condition_f', 'condition_detected' + self.name, self.machine.state_aborted],
            ['condition_s', 'condition_detected' + self.name, 'move_adjust' + self.name],

            ['check', 'move_adjust' + self.name, 'condition_done'],

            ['condition_f', 'condition_done' + self.name, 'condition_detected' + self.name],
            ['condition_s', 'condition_done' + self.name, self.machine.state_end],

        ]

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation)

    def conditioned_handler(self, event, direction):
        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0

        if direction == 'righter':
            value = event.righter()
        else:
            value = event.lefter()

        loginfo('\n')
        loginfo(f"DEBUG: current condition is {value}; event is {direction}; previous is {self.previous}")
        x, xs = event.get_better_track_x()
        loginfo(f"DEBUG: current center is at {x}; shift is at{xs}; angle might be {int(event.get_x_offset()*0.1)} \n")

        self.d_angle = int(event.get_x_offset()*0.1)

        if value == 1 and (direction == 'lefter' and self.previous == 'righter' or
                           direction == 'righter' and self.previous == 'lefter'):
            self.wobbles += 1
            loginfo(f"DEBUG: Cannot center precisely. Did {self.wobbles} wobbles")
        else:
            loginfo("DEBUG: Centering is going ok")

        if self.wobbles > self.give_up_threshold:
            loginfo("DEBUG: Cannot center precisely. Consider the centering is done")
            value = 0

        self.previous = direction
        return value

    def stabilize(self):
        self.reset_freeze()
        self.enable_object_detection(self.camera, True)
        self.machine.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.stabilize,
                'args': ()
            },
            'condition_detected' + self.name: {
                'condition': self.event_handler,
                'args': (self.gate_detected,)
            },
            'condition_righter' + self.name: {
                'condition': self.conditioned_handler,
                'args': (self.gate_detected, 'righter')
            },
            'condition_lefter' + self.name: {
                'condition': self.conditioned_handler,
                'args': (self.gate_detected, 'lefter')
            },
            'move_rotate' + self.name: {
                'march': 0.1,
                'lag': 0,
                'wait': 0.25,
                'yaw': self.d_angle
            },
        }
