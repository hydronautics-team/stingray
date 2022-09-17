from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectIsCloseEvent
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class AvoidSub(AUVMission):
    def __init__(self,
                 name: str,
                 camera: str,
                 avoid: list,
                 lag_dir='left',
                 confirmation: int = 2,
                 tolerance: int = 6,
                 ):
        self.name = '_' + name
        self.camera = camera
        self.avoid = avoid
        self.lag_dir = lag_dir
        self.confirmation = confirmation
        self.tolerance = tolerance
        self.avoid_states = tuple(f'condition_avoid_{i}' + self.name for i in self.avoid)
        if not self.avoid_states:
            raise TypeError('empty avoidance list given')
        super().__init__(name)

    def setup_events(self):
        self.avoid_assessions = [
                ObjectIsCloseEvent(get_objects_topic(self.camera), i, self.confirmation)
                for i in self.avoid
        ]

    def avoid_event_handler(self, event, *args, **kwargs):
        event.start_listening()
        rospy.sleep(1)
        if event.is_triggered():
            rospy.loginfo("DEBUG: Threat detected")
            event.stop_listening()
            return 1
        else:
            rospy.loginfo("DEBUG: No risks of collision with one of threatening objects")
            event.stop_listening()
            return 0

    def setup_states(self):
        states = ('move_lag' + self.name, ) + self.avoid_states
        return states

    def setup_transitions(self):
        t1 = [
            ['condition_s', self.avoid_states[i], 'move_lag' + self.name] for i in range(1, len(self.avoid))
        ] if len(self.avoid) > 1 else []

        t2 = [
            ['condition_f', self.avoid_states[i - 1], self.avoid_states[i]] for i in range(1, len(self.avoid)-1)
        ] if len(self.avoid) > 1 else []
        t3 = [
            ['condition_f', self.avoid_states[-1], self.machine.state_end]
        ] if len(self.avoid) > 1 else []
        t1 = t1 + t2 + t3
        transitions = t1 + [
                   [self.machine.transition_start, [self.machine.state_init,
                                                    'move_lag' + self.name], self.avoid_states[0]],

                   ['condition_f', self.avoid_states[0], self.avoid_states[1]] if len(self.avoid) > 1 else
                   ['condition_f', self.avoid_states[0], self.machine.state_end],
                   ['condition_s', self.avoid_states[0], 'move_lag' + self.name],

               ]
        return transitions

    def stabilize(self):
        self.enable_object_detection(self.camera, True)
        self.machine.auv.execute_move_goal({
                    'march': 0.0,
                    'lag': 0.0,
                    'yaw': 0,
                })

    def setup_scene(self):
        assession_scene = dict()
        for i in range(len(self.avoid)):
            assession_scene.update({
                f'{self.avoid_states[i]}': {
                    'condition': self.avoid_event_handler,
                    'args': (self.avoid_assessions[i],)
                },
            })
        scene = {
            self.machine.state_init: {
                'preps': self.stabilize,
                "args": (),
            },
            'move_lag' + self.name: {
                'march': 0,
                'lag': 0.25 if self.lag_dir == 'left' else -0.25,
                'yaw': 0,
                'wait': 0.1
            },
        }
        scene.update(assession_scene)
        return scene

    
