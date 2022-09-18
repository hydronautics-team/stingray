from stingray_tfsm.submachines.centering_on_move import CenteringOnMoveSub
from stingray_tfsm.submachines.avoid_submachine import AvoidSub
from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectIsCloseEvent
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class ReachOnMoveSub(AUVMission):
    def __init__(self,
                 name: str,
                 camera: str,
                 target: str,
                 avoid: list = [],
                 rotate='left',
                 lag='left',
                 confirmation: int = 2,
                 tolerance: int = 6,
                 vision: bool = True,
                 acoustics: bool = False,
                 speed: int = 0.5,
                 ):
        if target == 'yellow_flare':
            tolerance = 3
            confirmation = 1
        self.name = '_'+name
        self.camera = camera
        self.target = target
        self.speed = speed
        self.previous_center = (-1, -1)
        self.tolerance = tolerance
        self.confirmation = confirmation

        self.target_detection_event = None
        self.centering_submachine = CenteringOnMoveSub(
            name + "_centering", camera, target, tolerance=self.tolerance, confirmation=self.confirmation)
        self.centering_flare = CenteringOnMoveSub(
            name + "_centering", camera, 'red_flare', tolerance=self.tolerance, confirmation=self.confirmation)

        self.rotate_dir = -1 if rotate == "left" else 1
        self.target = target
        if avoid:
            self.avoid = True
            self.avoid_submachine = AvoidSub(
                name + "_avoid", camera, avoid, lag)
        super().__init__(name)

    def setup_states(self):
        states = ('condition_visible', 'move_march',
                  'move_search', 'condition_centering',
                  'custom_avoid', 'move_lag', 'condition_in_front'
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        if self.avoid:
            partial_transitions = [
                [self.machine.transition_start, [self.machine.state_init, ], 'custom_avoid' + self.name],

                ['avoid' + self.name, 'custom_avoid' + self.name, 'condition_visible' + self.name],

                ['condition_f', 'condition_in_front' + self.name, 'custom_avoid' + self.name],
            ]
        else:
            partial_transitions = [
                [self.machine.transition_start, [self.machine.state_init, ], 'condition_visible' + self.name],

                ['condition_f', 'condition_in_front' + self.name, 'condition_visible' + self.name],
            ]
        transitions = partial_transitions + [


            ['condition_f', 'condition_visible' + self.name, 'move_search' + self.name],
            ['condition_s', 'condition_visible' + self.name, 'condition_centering' + self.name],

            ['search' + self.name, 'move_search' + self.name, 'condition_visible' + self.name],

            ['condition_f', 'condition_centering' + self.name, 'condition_visible' + self.name],
            ['condition_s', 'condition_centering' + self.name, 'move_march' + self.name],

            ['next' + self.name, 'move_march' + self.name, 'condition_in_front' + self.name],


            ['condition_s', 'condition_in_front' + self.name, self.machine.state_end],
        ]
        return transitions

    def setup_scene(self):
        partial_scene = {'custom_avoid' + self.name: {
            'subFSM': True,
            'custom': self.avoid_submachine,
            'args': ()
        }, } if self.avoid else dict()
        scene = {
            self.machine.state_init: {
                'preps': self.enable_object_detection,
                "args": (self.camera, True),
            },
            'condition_visible' + self.name: {
                'condition': self.target_event_handler,
                'args': ()
            },
            'condition_in_front' + self.name: {
                'condition': self.arrival_handler,
                'args': ()
            },
            'move_search' + self.name: {
                'march': 0.0,
                'lag': 0,
                'yaw': 5 * self.rotate_dir,
                'wait': 0.3,
            },
            'condition_centering' + self.name: {
                'subFSM': True,
                'condition': self.centering_submachine,
                'args': ()
            },
            'move_march' + self.name: {
                'march': 0.6,
                'lag': 0.0,
                'yaw': 0,
                'wait': self.speed
            },
            'custom_avoid' + self.name: {
                'subFSM': True,
                'custom': self.avoid_submachine,
                'args': ()
            },
        }
        scene.update(partial_scene)
        return scene

    def setup_events(self):
        if self.target == "yellow_flare":
            self.confirmation = 0.20

        self.target_detection_event = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation)

    def target_event_handler(self):
        self.target_detection_event.start_listening()
        rospy.loginfo("DEBUG: started listening target detection")

        rospy.sleep(0.5)
        if self.target_detection_event.is_triggered():
            rospy.loginfo(f"DEBUG: target detected by event")
            rospy.loginfo(f"***\nDEBUG: target detected at {self.target_detection_event.get_track()}\n***")
            self.target_detection_event.stop_listening()
            return 1
        else:
            rospy.loginfo("DEBUG: no target detected")
            self.target_detection_event.stop_listening()
            return 0

    def arrival_handler(self):
        rospy.sleep(1)
        return not self.target_event_handler()

