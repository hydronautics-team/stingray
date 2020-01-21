#!/usr/bin/env python

import rospy
from stingray_msgs.srv import SetDeviceAction

import smach
import smach_ros

def scenario_1():
    sm = smach.StateMachine(outcomes=['DEMO_OK', 'DEMO_FAILED'])

    forward = SetDeviceAction

    forward.direction = MoveGoal.DIRECTION_FORWARD
    forward.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    forward.value = 10000

    rotation = MoveGoal()
    rotation.direction = MoveGoal.ROTATE_YAW_CCW
    rotation.velocityLevel = MoveGoal.VELOCITY_LEVEL_1
    rotation.value = 800

    with sm:
        smach.StateMachine.add('DELAY', common_states.WaitState(3), transitions={'OK': 'FORWARD_1'})

        smach.StateMachine.add('FORWARD_1',
                               smach_ros.SimpleActionState(
                                   'move_by_time',
                                   MoveAction,
                                   goal=forward),
                               {'succeeded': 'DEMO_OK', 'preempted': 'DEMO_FAILED', 'aborted': 'DEMO_FAILED'})
    return sm


if __name__ == '__main__':
    try:
        scenario_1()
    except rospy.ROSInterruptException: pass