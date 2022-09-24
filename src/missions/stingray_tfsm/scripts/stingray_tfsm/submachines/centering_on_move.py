from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectOnRight, ObjectOnLeft
from stingray_tfsm.auv_control import AUVControl
import rospy


class CenteringWithAvoidSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 auv: AUVControl,
                 camera: str,
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 20,
                 confidence: float = 0.3,
                 avoid: str = None,
                 avoid_confirmation: int = 2,
                 avoid_tolerance: int = 20,
                 avoid_confidence: float = 0.3,
                 verbose: bool = False,
                 speed: float = 0.5,
                 wait: int = 3,
                 lag='left',
                 is_big_value: float = 0.5,
                 ):
        """ Submission for centering on object in camera

        Args:
            name (str): mission name
            camera (str): camera name
            target (str): object name
            confirmation (int, optional): confirmation value of continuously detected object
             after which will be event triggered. Defaults to 2.
            tolerance (int, optional): centering tolerance. Defaults to 14.
        """
        self.name = name
        self.target = target
        self.confirmation = confirmation
        self.tolerance = tolerance
        self.confidence = confidence
        self.camera = camera
        self.avoid = avoid
        self.avoid_confirmation = avoid_confirmation
        self.avoid_tolerance = avoid_tolerance
        self.avoid_confidence = avoid_confidence
        self.speed = speed
        self.wait = wait
        self.is_big_value = is_big_value

        self.lag_dir = -1 if lag == "left" else 1


        self.target_detected = None
        self.avoid_detected = None

        super().__init__(name, auv, verbose)

    def setup_states(self):
        states = ('condition_detected', 'condition_centering',
                  'custom_stop', 'custom_avoid')
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, self.machine.state_init, 'custom_avoid' + self.name],

            ['do_avoid' + self.name, 'custom_avoid' + self.name, 'condition_centering' + self.name],

            ['condition_f', 'condition_centering' + self.name, 'custom_avoid' + self.name],
            ['condition_s', 'condition_centering' + self.name, self.machine.state_end],
        ]

    def run_detection(self):
        self.event_handler(self.target_detected, 0.5)
        if self.target_detected.is_triggered():
            rospy.loginfo(f'target_detected!')
            return True
        return False
    
    def run_avoid(self):
        if self.avoid is not None:
            rospy.loginfo(f'avoid')
            self.event_handler(self.avoid_detected, 0.5)
            if self.avoid_detected.is_triggered():
                rospy.loginfo(
                    f'self.avoid_detected.is_big() {self.avoid_detected.is_big_h()}')
                if self.avoid_detected.is_big_h():
                    self.do_avoid()
            else:
                rospy.loginfo(f'avoid not detected')

    def run_centering(self):
        self.event_handler(self.target_detected, 0.5)
        if self.target_detected.is_triggered():
            rospy.loginfo(
                f'self.target_detected.is_big() {self.target_detected.is_big_h()}')
            if self.target_detected.is_big_h():
                return True

            error = self.target_detected.get_x_offset()
            rospy.loginfo(f'error {error}')
            coef = int(error * 0.1)
            rospy.loginfo(f'set yaw {coef}')

            if abs(error) > self.tolerance:
                self.machine.auv.execute_move_goal({
                    'march': self.speed,
                    'lag': 0.0,
                    'yaw': coef,
                    'wait': self.wait,
                })

            return False

        self.machine.auv.execute_move_goal({
                    'march': self.speed,
                    'lag': 0.0,
                    'yaw': 0,
                })

    def do_avoid(self):
        rospy.loginfo('MINUS LAAAAAAAAAAAAAAAAAAAAAAAAAAAAG')
        self.machine.auv.execute_move_goal({
            'march': self.speed,
            'lag': 0.7 * self.lag_dir,
            'yaw': 0,
            'wait': 4,
        })
        # rospy.loginfo('LAAAAAAAAAAAAAAAAAAAAAAAAAAAAG')
        # self.machine.auv.execute_move_goal({
        #     'march': self.speed,
        #     'lag': -0.7 * self.lag_dir,
        #     'yaw': 0,
        #     'wait': 3,
        # })

    def prerun(self):
        pass

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.prerun,
                "args": (),
            },
            'custom_avoid' + self.name: {
                'custom': self.run_avoid,
                'args': ()
            },
            'condition_centering' + self.name: {
                'condition': self.run_centering,
                'args': ()
            },
        }

    def setup_events(self):
        self.target_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation, confidence=self.confidence, is_big_h_value=0.5)
        if self.avoid is not None:
            self.avoid_detected = ObjectDetectionEvent(
                get_objects_topic(self.camera), self.avoid, self.avoid_confirmation, confidence=self.avoid_confidence, is_big_h_value=0.3)
