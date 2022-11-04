#! /usr/bin/env python3

from stingray_tfsm.auv_mission import AUVMission
from stingray_object_detection.utils import get_objects_topic
from stingray_tfsm.vision_events import ObjectDetectionEvent, ObjectIsCloseEvent
from stingray_tfsm.auv_control import AUVControl
import rospy


class TestMission(AUVMission):
    def __init__(self,
                 name: str,
                 auv: AUVControl = None,
                 ):
        super().__init__(name, auv)

    def setup_states(self):
        return ('dive_0', 'move_march_1', 'move_yaw_1',
                'move_march_2', 'move_yaw_2',
                'move_march_3', 'move_yaw_3',
                'move_march_4', 'move_yaw_4',)

    def setup_transitions(self):
        return [
            [self.machine.transition_start, self.machine.state_init, 'dive_0'],
            ['step_0', 'dive_0', 'move_march_1'],
            # ['step_11', 'move_march_1', 'move_march_1'],
            ['step_11', 'move_march_1', 'move_yaw_1'],

            ['step_21', 'move_yaw_1', 'move_march_2'],
            ['step_22', 'move_march_2', 'move_yaw_2'],

            ['step_31', 'move_yaw_2', 'move_march_3'],
            ['step_32', 'move_march_3', 'move_yaw_3'],

            ['step_41', 'move_yaw_3', 'move_march_4'],
            ['step_42', 'move_march_4', 'move_yaw_4'],

            ['step_42', 'move_yaw_4', 'move_march_1'],

        ]

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'time': 2,
            },
            'dive_0': {
                'depth': 1800,
                'check_depth': True

            },
            'move_march_1': {
                'march': 0.7,
                'lag': 0.0,
                'yaw': 0,
                'wait': 45,
            },
            'move_yaw_1': {
                'march': 0.0,
                'lag': 0.0,
                'yaw': 90,
                'wait': 6,
                # 'check_yaw': True
            },
            'move_march_2': {
                'march': 0.7,
                'lag': 0.0,
                'yaw': 0,
                'wait': 30,
            },
            'move_yaw_2': {
                'march': 0.0,
                'lag': 0.0,
                'yaw': 90,
                'wait': 6,
                # 'check_yaw': True
            },
            'move_march_3': {
                'march': 0.7,
                'lag': 0.0,
                'yaw': 0,
                'wait': 15,
            },
            'move_yaw_3': {
                'march': 0.0,
                'lag': 0.0,
                'yaw': 90,
                'wait': 6,
                # 'check_yaw': True
            },
            'move_march_4': {
                'march': 0.7,
                'lag': 0.0,
                'yaw': 0,
                'wait': 30,
            },
            'move_yaw_4': {
                'march': 0.0,
                'lag': 0.0,
                'yaw': 90,
                'wait': 6,
                # 'check_yaw': True
            },
        }

    def setup_events(self):
        pass
