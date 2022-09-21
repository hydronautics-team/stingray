from stingray_tfsm.submachines.centering_on_move import CenteringOnMoveSub
from stingray_tfsm.submachines.avoid_submachine import AvoidSub
from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectIsCloseEvent
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.auv_control import AUVControl
import rospy


class ReachOnMoveSub(AUVMission):
    def __init__(self,
                 name: str,
                 auv: AUVControl,
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
                 verbose: bool = False,
                 ):
        self.name = name
        self.camera = camera
        self.target = target
        self.speed = speed
        self.tolerance = tolerance
        self.confirmation = confirmation

        self.centering_gate = CenteringOnMoveSub(
            name + "_centering_gate", auv, camera, target, tolerance=self.tolerance, confirmation=self.confirmation, confidence=0.3)
        self.centering_flare = CenteringOnMoveSub(
            name + "_centering_flare", auv, camera, 'red_flare', tolerance=self.tolerance, confirmation=self.confirmation, confidence=0.5)

        super().__init__(name, auv, verbose)

    def setup_states(self):
        states = ('condition_centering_flare', 'custom_avoid',
                  'condition_centering_gate', 'move_march',
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        transitions =  [
            [self.machine.transition_start, self.machine.state_init, 'condition_centering_flare' + self.name],

            ['condition_f', 'condition_centering_flare' + self.name, 'condition_centering_flare' + self.name],
            ['condition_s', 'condition_centering_flare' + self.name, 'custom_avoid' + self.name],

            ['go_gate' + self.name, 'custom_avoid' + self.name, 'condition_centering_gate' + self.name],

            ['condition_f', 'condition_centering_gate' + self.name, 'condition_centering_gate'],
            ['condition_s', 'condition_centering_gate' + self.name, self.machine.state_end],

            # [self.machine.transition_end, 'move_march' + self.name, self.machine.state_end],
        ]

        return transitions

    def avoid_custom(self):
        # rospy.loginfo('STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP')
        # self.machine.auv.execute_move_goal({
        #     'march': 0.0,
        #     'lag': 0.0,
        #     'yaw': 0,
        #     'wait': 5,
        # })
        rospy.loginfo('MINUS LAAAAAAAAAAAAAAAAAAAAAAAAAAAAG')
        self.machine.auv.execute_move_goal({
            'march': 0.3,
            'lag': -0.7,
            'yaw': 0,
            'wait': 6,
        })
        # rospy.loginfo('STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP')
        # self.machine.auv.execute_move_goal({
        #     'march': 0.0,
        #     'lag': 0.0,
        #     'yaw': 0,
        #     'wait': 5,
        # })
        rospy.loginfo('LAAAAAAAAAAAAAAAAAAAAAAAAAAAAG')
        self.machine.auv.execute_move_goal({
            'march': 0.3,
            'lag': 0.7,
            'yaw': 0,
            'wait': 3,
        })
        # rospy.loginfo('STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP')
        # self.machine.auv.execute_move_goal({
        #     'march': 0.0,
        #     'lag': 0.0,
        #     'yaw': 0,
        #     'wait': 5,
        # })
        # rospy.loginfo('YAAAAAAAAAAAAAAAAAAAAAAAAAAAAW')
        # self.machine.auv.execute_move_goal({
        #     'march': 0.0,
        #     'lag': 0.0,
        #     'yaw': 10,
        # })

    def setup_scene(self):
        scene = {
            self.machine.state_init: {
                'time': 0.1,
            },
            'condition_centering_flare' + self.name: {
                'subFSM': True,
                'condition': self.centering_flare,
                'args': ()
            },
            'custom_avoid' + self.name: {
                'custom': self.avoid_custom,
                'args': ()
                
            },
            'condition_centering_gate' + self.name: {
                'subFSM': True,
                'condition': self.centering_gate,
                'args': ()
            },
        }
        return scene

    def setup_events(self):
        pass 
    