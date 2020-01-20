#! /usr/bin/env python

import rospy
import smach
import smach_ros

from states.special_states import InitializationState


def main():
    rospy.init_node("control_fsm")

    delay_after_init = rospy.get_param("~delay_after_init", None)
    if delay_after_init is None:
        rospy.logerr("Parameter 'delay_after_init' must be specified!")
        return
    introspection_needed = rospy.get_param("~introspection", False)

    state_machine = smach.StateMachine(outcomes=["CONTROL_FSM_SUCCEEDED", "CONTROL_FSM_FAILED"])

    with state_machine:
        smach.StateMachine.add("INITIALIZATION", InitializationState(delay_after_init),
                               transitions={"INIT_OK": "CONTROL_FSM_SUCCEEDED"})

    server = None
    if introspection_needed:
        server = smach_ros.IntrospectionServer('control_fsm', state_machine, '/stingray/fsm')
        server.start()

    rospy.loginfo("Starting control FSM")
    try:
        outcome = state_machine.execute()
    except Exception as e:
        rospy.logerr("Control FSM failed with exception: " + e.message)

    rospy.spin()
    if server is not None:
        server.stop()

    rospy.loginfo("Control FSM finished")


if __name__ == '__main__':
    main()