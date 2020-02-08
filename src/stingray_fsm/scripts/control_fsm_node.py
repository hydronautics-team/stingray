#! /usr/bin/env python

import rospy
import smach
import smach_ros

from states.special_states import InitializationState


def main():
    rospy.init_node("control_fsm")

    missions_fsm_module_name = rospy.get_param("~missions_module", None)
    if missions_fsm_module_name is None:
        rospy.logerr("Parameter 'missions_module' must be specified!")
        return

    missions_fsm_class_name = rospy.get_param("~missions_class", None)
    if missions_fsm_class_name is None:
        rospy.logerr("Parameter 'missions_class' must be specified!")
        return

    delay_after_init = rospy.get_param("~delay_after_init", None)
    if delay_after_init is None:
        rospy.logerr("Parameter 'delay_after_init' must be specified!")
        return

    introspection_needed = rospy.get_param("~introspection", False)

    state_machine = smach.StateMachine(outcomes=["CONTROL_FSM_SUCCEEDED", "CONTROL_FSM_FAILED"])

    with state_machine:
        smach.StateMachine.add("INITIALIZATION", InitializationState(delay_after_init),
                               transitions={"INIT_OK": "MISSIONS"})

        missions_module = __import__(missions_fsm_module_name)
        missions_class = getattr(missions_module, missions_fsm_class_name)
        missions_instance = missions_class()

        smach.StateMachine.add("MISSIONS", missions_instance,
                               transitions={"MISSIONS_OK": "CONTROL_FSM_SUCCEEDED",
                                            "MISSIONS_FAILED": "CONTROL_FSM_FAILED"})


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
