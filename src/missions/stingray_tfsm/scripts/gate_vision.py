from stingray_tfsm.ros_transitions import AUVStateMachine
from stingray_object_detection_msgs import ObjectsArray
import rospy

HARDCODE = '/rov_model_urdf/camera_front/image_raw/yolo_detector/objects'
EXHAUST_MAX = 40
CONFIRMATION = 3
STATES = ('init', 'condition_gate', 'rotate_clockwise', 'move_march', 'done', 'aborted')
TRANSITIONS = [     # Vision exhaustion loop
    ['start', ['init', 'rotate_clockwise', 'move_march'], 'condition_gate'],
    ['condition_f', 'condition_gate', 'rotate_clockwise'],
    ['condition_s', 'condition_gate', 'move_march'],
    ['end', '*', 'done']
]


def are_gates_visible():

    return 1


SCENE = {
    'init': {
        'time': 60
    },
    'condition_gate': {
        'condition': are_gates_visible,
        'args': HARDCODE
    },
    'rotate_clockwise': {
        'angle': 10
    },
    'move_march': {
        'direction': 3,
        'velocity': 0.4,
        'duration': 2000
    },
    'done': {},
    'aborted': {}
}

gate_mission = AUVStateMachine(STATES, TRANSITIONS, SCENE)

if __name__ == '__main__':
    rospy.init_node("control_fsm")
    gate_mission.run()
