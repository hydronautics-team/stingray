from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.vision_events import ObjectDetectionEvent
from stingray_object_detection.utils import get_objects_topic
from rospy import loginfo, sleep
from stingray_tfsm.auv_control import AUVControl


class CenteringPlanarSub(AUVMission):
    """ Submission for centering on object in camera """

    def __init__(self, name: str,
                 camera: str,  # bottom needed
                 target: str,
                 confirmation: int = 2,
                 tolerance: int = 9,
                 auv: AUVControl = None,
                 dropper_offset_x=-150,
                 simulation=False,
                 ):
        """ Submission for centering on object in camera

        Args:
            name (str): mission name
            camera (str): camera name
            target (str): object name
            confirmation (int, optional): confirmation value of continuously detected object after which will be event triggered. Defaults to 2.
            tolerance (int, optional): centering tolerance. Defaults to 14.
        """
        self.name = '_' + name
        self.target = target
        self.confirmation = confirmation
        self.tolerance = tolerance
        self.camera = camera
        self.previous = 0  # or 1
        self.wobbles = 0
        self.give_up_threshold = 15
        self.move_speed = 0.2
        self.lag_time = 0
        self.march_time = 0
        self.dropper_offset_x = dropper_offset_x
        self.simulation = simulation
        self.done = False

        if auv is None:
            self.auv = AUVControl(verbose=False)  # govnocod
        else:
            self.auv = auv

        self.gate_detected = None
        super().__init__(name)

    def reset_freeze(self):
        self.wobbles = 0
        self.done = False

    def setup_states(self):
        states = (
            'condition_detected',
            'move_stop',
            'move_lag',
            'move_march',
            'custom_lag_wait',
            'custom_march_wait',
            'move_stop_abort'
                  )
        states = tuple(i + '_' + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [
                self.machine.state_init, 'custom_lag_wait' + '_' + self.name], 'condition_detected' + '_' + self.name],

            ['march_stop', 'move_march' + '_' + self.name, 'custom_march_wait' + '_' + self.name],

            ['lag_go', 'custom_march_wait' + '_' + self.name, 'move_lag' + '_' + self.name],
            ['lag_stop', 'move_lag' + '_' + self.name, 'custom_lag_wait' + '_' + self.name],

            ['condition_f', 'condition_detected' + '_' + self.name, 'move_stop_abort' + '_' + self.name],
            ['condition_s', 'condition_detected' + '_' + self.name, 'move_march' + '_' + self.name],


        ]

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation, confidence=0.2)

    def stabilize(self):
        self.reset_freeze()
        self.enable_object_detection(self.camera, True)
        self.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })
        loginfo("RESET THIS FUCKING DEPTH WHILE NOT TOO LATE")
        # sleep(5)
        loginfo("LETZZ FUCKING GO")

    def calculate_offsets(self, event):
        if self.done:
            loginfo("Already centered")
            return 0

        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0

        if self.simulation:
            loginfo(f"it's simulator and image is not upright")
            self.lag_time = int(event.get_y_offset() * 0.02)
            self.march_time = int(event.get_x_offset() * 0.02)
            loginfo(f'shifts are {event.get_y_offset()}px right and {event.get_x_offset()}px forward')
        else:
            self.lag_time = int(event.get_x_offset() * 0.02)
            self.march_time = int(event.get_y_offset() * 0.02)

        if self.target == 'red_bowl' or self.target == 'blue_bowl':
            if (event.get_y_offset() * 0.02)**2+(event.get_x_offset() * 0.02)**2 < 4:
                loginfo('DOOOOOONEEEEEE')
                self.done = True

        loginfo(f'it looks like we need to move {self.lag_time}s right and {self.march_time}s forward')

        return 1

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.stabilize,
                'args': ()
            },
            'move_march' + '_' + self.name: {
                'march': self.move_speed if self.march_time >= 0 else -self.move_speed,
                'lag': 0.0,
                'yaw': 0,
            },
            'move_lag' + '_' + self.name: {
                'march': 0.0,
                'lag': self.move_speed if self.lag_time < 0 else -self.move_speed,
                'yaw': 0,
            },
            'move_stop_abort' + '_' + self.name: {
                'march': 0,
                'lag': 0.0,
                'yaw': 0,
                'wait': 1
            },
            'custom_lag_wait' + '_' + self.name: {
                'custom': sleep,
                'args': (abs(self.lag_time),)
            },
            'custom_march_wait' + '_' + self.name: {
                'custom': sleep,
                'args': (abs(self.march_time),)
            },
            'condition_detected' + '_' + self.name: {
                'condition': self.calculate_offsets,
                'args': (self.gate_detected,)
            },

        }