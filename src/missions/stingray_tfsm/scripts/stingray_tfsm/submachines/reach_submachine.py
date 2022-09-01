from stingray_tfsm.submachines.centering_angle import CenteringAngleSub
from stingray_tfsm.submachines.avoid_submachine import AvoidSub
from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectIsCloseEvent
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class ReachSub(AUVMission):
    def __init__(self,
                 name: str,
                 front_camera: str,
                 bottom_camera: str,
                 target: str,
                 avoid: list,
                 rotate='left',
                 lag='left'):
        if target == 'yellow_flare':
            tolerance = 3
            confirmation = 1
        else:
            tolerance = self.tolerance
            confirmation = self.confirmation
        self.centering_submachine = CenteringAngleSub(
            "centering", front_camera, target, tolerance=tolerance, confirmation=confirmation)
        self.rotate_dir = 1 if rotate == "left" else -1
        self.target = target
        if avoid:
            self.avoid = True
            self.avoid_submachine = AvoidSub(
                "avoid", front_camera, bottom_camera, avoid, lag)
        super().__init__(name, front_camera, bottom_camera)

    def setup_states(self):
        return ('condition_visible', 'move_march',
                'rotate_search', 'condition_centering',
                'custom_avoid', 'move_lag', 'condition_in_front'
                )

    def setup_transitions(self):
        return [
                   [self.machine.transition_start, [self.machine.state_init, ],
                    'custom_avoid'],

                   ['avoid', 'custom_avoid', 'condition_visible'],

                   ['condition_f', 'condition_visible', 'rotate_search'],
                   ['condition_s', 'condition_visible', 'condition_centering'],

                   ['search', 'rotate_search', 'condition_visible'],

                   ['condition_f', 'condition_centering', 'condition_visible'],
                   ['condition_s', 'condition_centering', 'move_march'],

                   ['next', 'move_march', 'condition_in_front'],

                   ['condition_f', 'condition_in_front', 'custom_avoid'],
                   ['condition_s', 'condition_in_front', self.machine.state_end],
               ] + self.machine.default_transitions

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.enable_object_detection,
                "args": (self.front_camera, True),
            },
            'condition_visible': {
                'condition': self.target_event_handler,
                'args': ()
            },
            'condition_in_front': {
                'condition': self.arrival_handler,
                'args': ()
            },
            'rotate_search': {
                'angle': 5 * self.rotate_dir
            },
            'condition_centering': {
                'subFSM': True,
                'condition': self.centering_submachine,
                'args': ()
            },
            'move_march': {
                'direction': 3,
                'velocity': 0.4,
                'duration': 1500
            },
            'custom_avoid': {
                'subFSM': True,
                'custom': self.avoid_submachine,
                'args': ()
            },
        }

    def setup_events(self):
        if self.target == "yellow_flare":
            self.confirmation = 0.20

        self.target_detection_event = ObjectDetectionEvent(
            get_objects_topic(self.front_camera), self.target, self.confirmation )

    def target_event_handler(self):
        self.target_detection_event.start_listening()
        rospy.loginfo("DEBUG: started listening target detection")

        rospy.sleep(0.5)
        if self.target_detection_event.is_triggered():
            rospy.loginfo("DEBUG: target detected by event")
            self.target_detection_event.stop_listening()
            return 1
        else:
            rospy.loginfo("DEBUG: no target detected")
            self.target_detection_event.stop_listening()
            return 0

    def arrival_handler(self):
        rospy.sleep(1)
        return not self.target_event_handler()

    def check_machine(self):
        if type(self.machine) is AUVStateMachine or \
                type(self.machine) is PureStateMachine:
            return 1
        else:
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
