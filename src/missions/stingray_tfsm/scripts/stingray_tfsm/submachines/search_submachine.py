from stingray_tfsm.submachines.centering_angular import CenteringAngleSub
from stingray_tfsm.submachines.avoid_submachine import AvoidSub
from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_control import AUVControl
import rospy


class SearchSub(AUVMission):
    def __init__(self,
                 name: str,
                 auv: AUVControl,
                 camera: str,
                 target: str,
                 avoid: list = [],
                 rotate='left',
                 lag='left',
                 confirmation: int = 2,
                 tolerance: int = 8,
                 confidence: float = 0.3,
                 vision: bool = True,
                 acoustics: bool = False,
                 speed: int = 0.5,
                 ):
        self.name = name
        self.camera = camera
        self.target = target
        self.speed = speed
        self.previous_center = (-1, -1)
        self.tolerance = tolerance
        self.confirmation = confirmation
        self.confidence = confidence

        self.target_detection_event = None

        self.rotate_dir = -1 if rotate == "left" else 1
        self.target = target

        super().__init__(name, auv)

    def setup_states(self):
        states = ('condition_visible',
                  'move_search',
                  'condition_centering',
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        transitions = [
            [self.machine.transition_start, self.machine.state_init,
                'condition_visible' + self.name],

            ['condition_f', 'condition_visible' +
                self.name, 'move_search' + self.name],
            ['condition_s', 'condition_visible' +
                self.name, self.machine.state_end],

            ['search' + self.name, 'move_search' +
                self.name, 'condition_visible' + self.name],
        ]
        return transitions

    def setup_scene(self):
        scene = {
            self.machine.state_init: {
                'preps': print,
                "args": (),
            },
            'condition_visible' + self.name: {
                'condition': self.event_handler,
                'args': (self.target_detection_event, 0.5)
            },
            'move_search' + self.name: {
                'march': 0.0,
                'lag': 0,
                'yaw': 7 * self.rotate_dir,
                'wait': 0.5,
            },
        }
        return scene

    def setup_events(self):
        self.target_detection_event = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation, confidence=self.confidence)


