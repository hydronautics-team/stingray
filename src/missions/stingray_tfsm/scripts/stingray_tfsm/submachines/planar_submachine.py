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
                 lifter_offset_x=-20,
                 simulation=True,
                 drop=True
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
        if drop:
            self.lifter_offset_x = -100

        else:
            self.lifter_offset_x = lifter_offset_x
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
            'move_stop_abort',
            'custom_adjust',
                  )
        states = tuple(i + '_' + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [
                self.machine.state_init,
                'custom_adjust' + '_' + self.name
            ], 'condition_detected' + '_' + self.name],

            ['condition_f', 'condition_detected' + '_' + self.name, 'move_stop_abort' + '_' + self.name],
            ['condition_s', 'condition_detected' + '_' + self.name, 'custom_adjust' + '_' + self.name],


        ]

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation, confidence=0.2, closest=True)

    def stabilize(self):
        self.reset_freeze()
        self.enable_object_detection(self.camera, True)
        self.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })
        loginfo("LETZZ FUCKING GO")

    def calculate_offsets(self, event):
        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0

        if self.simulation:
            x_offset = event.get_y_offset()
            y_offset = event.get_x_offset()

            loginfo(f"it's simulator and image is not upright")
        else:
            x_offset = event.get_x_offset()
            y_offset = event.get_y_offset()

        lag_dist = (x_offset + self.lifter_offset_x)
        self.lag_time = int(lag_dist * 0.02)
        march_dist = (y_offset - 10)
        self.march_time = int(march_dist * 0.02)
        loginfo(f'shifts are {lag_dist}px right and {march_dist}px forward')

        if self.target == 'red_bowl' or self.target == 'blue_bowl':
            if (lag_dist * 0.1)**2+(march_dist * 0.1)**2 < 1:
                loginfo('DOOOOOONEEEEEE')
                self.done = True

        if self.done:
            loginfo("Already centered")
            return 0

        loginfo(f'it looks like we need to move {self.lag_time}s right and {self.march_time}s forward')

        return 1

    def adjust(self):
        if self.march_time:
            self.auv.execute_move_goal({  # go march
                    'march': self.move_speed if self.march_time >= 0 else -self.move_speed,
                    'lag': 0.0,
                    'yaw': 0,
                })
            sleep(abs(self.march_time))
        if self.lag_time:
            self.auv.execute_move_goal({  # go lag
                    'march': 0.0,
                    'lag': self.move_speed if self.lag_time >= 0 else -self.move_speed,
                    'yaw': 0,
                })
            sleep(abs(self.lag_time))
        if not(self.lag_time or self.march_time):
            self.done = 1
        self.auv.execute_move_goal({  # go lag
            'march': 0.0,
            'lag': 0,
            'yaw': 0,
        })

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.stabilize,
                'args': ()
            },
            'move_stop_abort' + '_' + self.name: {
                'march': 0,
                'lag': 0.0,
                'yaw': 0,
                'wait': 1
            },
            'condition_detected' + '_' + self.name: {
                'condition': self.calculate_offsets,
                'args': (self.gate_detected,)
            },
            'custom_adjust' + '_' + self.name: {
                'custom': self.adjust,
                'args': ()
            },
        }
