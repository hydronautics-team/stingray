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
                 simulation=False,
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
            'custom_stop',
            'move_lag',
            'move_march',
            'custom_lag_wait',
            'custom_march_wait',
            'move_stop_abort',
            'custom_adjust',

                  )
        states = tuple(i + '_' + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [
                self.machine.state_init,
            #    'custom_march_wait' + '_' + self.name,
            #    'custom_stop' + '_' + self.name,
                'custom_adjust' + '_' + self.name
            ], 'condition_detected' + '_' + self.name],

            #['march', 'move_march' + '_' + self.name, 'custom_march_wait' + '_' + self.name],

            # ['lag_go', 'custom_lag_wait' + '_' + self.name, 'move_march' + '_' + self.name],
            # ['lag_stop', 'move_lag' + '_' + self.name, 'custom_lag_wait' + '_' + self.name],
            #
            # ['STAHP', 'custom_lag_wait' + '_' + self.name, 'custom_stop' + '_' + self.name],

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
        loginfo("RESET THIS FUCKING DEPTH WHILE NOT TOO LATE")
        sleep(5)
        loginfo("LETZZ FUCKING GO")

    def calculate_offsets(self, event):
        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0

        if self.simulation:
            loginfo(f"it's simulator and image is not upright")
            self.lag_dist = (event.get_y_offset() + self.lifter_offset_x)
            self.lag_time = int(self.lag_dist * 0.02)
            self.march_dist = (event.get_x_offset()-10)
            self.march_time = int(self.march_dist * 0.02)
            loginfo(f'shifts are {self.lag_dist}px right and {self.march_dist}px forward')

        else:
            self.lag_dist = (event.get_x_offset()+30)
            self.lag_time = int(self.lag_dist * 0.02)
            self.march_dist = (event.get_y_offset() - 30)
            self.march_time = int(self.march_dist * 0.02)
            loginfo(f'shifts are {self.lag_dist}px right and {self.march_dist}px forward')

        if self.target == 'red_bowl' or self.target == 'blue_bowl':
            if (self.lag_dist * 0.1)**2+(self.march_dist * 0.1)**2 < 1:
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

    def stahp_pls(self):
        for i in range(1):
            self.auv.execute_move_goal(
                {
                    'march': 0.00,
                    'lag': 0.0000,
                    'yaw': 0,
                },
            )
            sleep(1)

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
                'lag': self.move_speed if self.lag_time >= 0 else -self.move_speed,
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
            'custom_stop' + '_' + self.name: {
                'custom': self.stahp_pls,
                'args': ()
            },
            'custom_adjust' + '_' + self.name: {
                'custom': self.adjust,
                'args': ()
            },
        }
