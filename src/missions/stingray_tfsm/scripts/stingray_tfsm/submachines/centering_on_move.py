from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectOnRight, ObjectOnLeft
import rospy

class CenteringOnMoveSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 camera: str,
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 20,
                 confidence: float = 0.3,
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
        self.confidence = confidence
        self.camera = camera

        self.gate_detected = None
        self.gate_lefter = None
        self.gate_righter = None
        super().__init__(name)

    def setup_states(self):
        states = ('condition_centering')
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [self.machine.state_init], 'condition_centering' + self.name],

            ['condition_f', 'condition_centering' + self.name, 'condition_centering' + self.name],
            ['condition_s', 'condition_centering' + self.name, self.machine.state_end],
        ]

    def prep(self):
        pass
        # self.enable_object_detection(self.camera, True)
        # self.machine.auv.execute_dive_goal({
        #             'depth': 1100,
        #         })
        # self.machine.auv.execute_move_goal({
        #     'march': 1.0,
        #     'lag': 0.0,
        #     'yaw': 0,
        #     'wait': 5,
        # })

    def run_centering(self):
        if self.event_handler(self.gate_detected):
            rospy.loginfo(f'self.gate_detected.is_big() {self.gate_detected.is_big()}')
            if self.gate_detected.is_big():
                return True
            
            current_center = self.gate_detected.get_track()
            error = current_center - 320
            rospy.loginfo(f'current_center {current_center}')
            rospy.loginfo(f'error {error}')
            coef = int(error * 0.1)
            rospy.loginfo(f'set yaw {coef}')

            if abs(error) > self.tolerance:
                self.machine.auv.execute_move_goal({
                    'march': 1.0,
                    'lag': 0.0,
                    'yaw': coef,
                    'wait': 5,
                })

            return False 

        
    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.prep,
                "args": (),
            },
            'condition_centering' + self.name: {
                'condition': self.run_centering,
                'args': ()
            },
        }

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation, confidence=self.confidence)