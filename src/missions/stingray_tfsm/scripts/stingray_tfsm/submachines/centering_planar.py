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
                 tolerance: int = 9
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
        self.move_speed = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.auv = AUVControl(verbose=False)  # govnocod

        self.gate_detected = None
        super().__init__(name)

    def reset_freeze(self):
        self.wobbles = 0

    def setup_states(self):
        states = ('condition_detected', 'condition_done',
                  'custom_lag', 'custom_march'
                  )
        states = tuple(i + self.name for i in states)
        return states

    def setup_transitions(self):
        return [
            [self.machine.transition_start, [self.machine.state_init], 'condition_detected' + self.name],

            ['condition_f', 'condition_detected' + self.name, self.machine.state_aborted],
            ['condition_s', 'condition_detected' + self.name, 'custom_march' + self.name],

            ['move', 'custom_march' + self.name, 'custom_lag'+ self.name],
            ['check', 'custom_lag' + self.name, 'condition_done'+ self.name],

            ['condition_f', 'condition_done' + self.name, 'condition_detected' + self.name],
            ['condition_s', 'condition_done' + self.name, self.machine.state_end],

        ]

    def setup_events(self):
        self.gate_detected = ObjectDetectionEvent(
            get_objects_topic(self.camera), self.target, self.confirmation)

    def calculate_offsets(self, event):
        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0

        self.x_offset = int(event.get_x_offset()*0.05)
        self.y_offset = int(event.get_y_offset()*0.05)

        return 1

    def ya_ebu(self, scene):
        self.auv.execute_move_goal(scene)

    def stabilize(self):
        self.reset_freeze()
        self.enable_object_detection(self.camera, True)
        self.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })
        loginfo("RESET THIS FUCKING DEPTH WHILE NOT TOO LATE")
        sleep(3)
        loginfo("LETZZ FUCKING GO")


    def check_done(self, event):
        if not self.event_handler(event, wait=0.5):
            self.reset_freeze()
            loginfo('dich happened')
            return 0
        if not(event.lefter() or event.righter() or event.higher() or event.lower()):
            loginfo(f"centering completed with tolerance {self.tolerance}%")
            return 1
        else:
            loginfo(f"--===CENTERING===--\n"
                    f"lefter: {event.lefter()}\t higher: {event.higher()}\n"
                    f"righter: {event.righter()}\t lower: {event.lower()}")
            return 0

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.stabilize,
                'args': ()
            },
            'condition_detected' + self.name: {
                'condition': self.calculate_offsets,
                'args': (self.gate_detected,)
            },
            'condition_done' + self.name: {
                'condition': self.check_done,
                'args': (self.gate_detected,)
            },
            'custom_lag' + self.name: {
                'custom': self.ya_ebu,
                'args': ({
                    'march': 0.0,
                    'lag': self.move_speed if self.x_offset > 0 else -self.move_speed,
                    'wait': 1,
                    'yaw': 0
                },)
            },
            'custom_march' + self.name: {
                'custom': self.ya_ebu,
                'args': ({
                    'march': self.move_speed if self.y_offset > 0 else -self.move_speed,
                    'lag': 0,
                    'wait': 1,
                    'yaw': 0
                },)
            },
        }
