#! /usr/bin/env python3

from stingray_tfsm.ros_transitions import AUVStateMachine
from stingray_tfsm.event import ObjectDetectionEvent
import rospy

HARDCODE = '/rov_model_urdf/camera_front/image_raw/yolo_detector/objects'
EXHAUST_MAX = 40
CONFIRMATION = 3
TARGET = 'gate'


class GateRighterEvent(ObjectDetectionEvent):
    pass


STATES = ('init', 'condition_gate_lefter', 'condition_gate_righter',
          'rotate_clockwise', 'rotate_anticlockwise', 'move_march', 'done', 'aborted')
TRANSITIONS = [     # Vision exhaustion loop
    ['start', ['init', 'rotate_clockwise', 'move_march'], 'condition_gate'],
    ['condition_f', 'condition_gate', 'rotate_clockwise'],
    ['condition_s', 'condition_gate', 'move_march'],
    ['end', '*', 'done']
]

gate_detection_event = ObjectDetectionEvent(HARDCODE, TARGET, CONFIRMATION)
EXHAUSTION = 0


def gates_condition(*args, **kwargs):
    global EXHAUSTION
    gate_detection_event.start_listening()
    rospy.sleep(0.1)
    if gate_detection_event.is_triggered():
        EXHAUSTION = 0
        return 1
    else:
        EXHAUSTION += 1
        if EXHAUSTION >= EXHAUST_MAX:
            rospy.loginfo("it's time to stop, but i'll implement it later")
        return 0


def rotation_overflow_condition(*args, **kwargs):
    return 1


STATES_ARGS = {
    'init': {
        'time': 10
    },
    'condition_gate': {
        'condition': gates_condition,
        'args': None
    },
    'rotate_clockwise': {
        'angle': 10
    },
    'rotate_anticlockwise': {
            'angle': -10
        },
    'move_march': {
        'direction': 3,
        'velocity': 0.4,
        'duration': 1000
    },
    'done': {},
    'aborted': {}
}

gate_mission = AUVStateMachine(STATES, TRANSITIONS, STATES_ARGS)

if __name__ == '__main__':
    rospy.init_node("control_fsm")
    gate_mission.run()
