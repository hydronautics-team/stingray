#! /usr/bin/env python3

import rospy
import roslaunch
import std_msgs.msg
import std_srvs.srv


launched = False
requestedMode = None
launchRequested = False
stopRequested = False


def barcode_callback(msg):
    global launched
    global launchRequested
    global stopRequested
    global requestedMode

    message = msg.data.lower()
    if not (message in ['demo', 'stop', 'qualification', 'simple', 'medium', 'medium_ha', ]
            or message.startswith('missions')):
        rospy.logerr('Unknown messages')
        return

    if message == 'stop':
        if not launched:
            rospy.logwarn('Not launched yet')
            return
        if stopRequested:
            rospy.logwarn('Stop is already requested')
            return
        stopRequested = True
        return

    if launched:
        rospy.logwarn('Already launched')
        return
    if launchRequested:
        rospy.logwarn('Launch is already requested')
        return

    requestedMode = message
    launchRequested = True


def stop_notification_callback(req):
    global stopRequested
    stopRequested = True
    return std_srvs.srv.TriggerResponse(True, 'OK')

def main():
    global launched
    global launchRequested
    global stopRequested
    global requestedMode

    rospy.init_node('qr_trigger')

    launch = None

    rospy.Subscriber('/barcode', std_msgs.msg.String, barcode_callback)
    rospy.Service('global_fsm_finished', std_srvs.srv.Trigger, stop_notification_callback) # я не зню что это за строчка

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo('barcode loope 1******************************')
        if launchRequested:
            rospy.loginfo('barcode loope 2222222222******************************')
            if requestedMode in ['qualification_simple', 'qualification_vision', 'demo', 'stop']:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nvidia/AUV/src/auv_startup/launch/" +
                #                                                  requestedMode + ".launch"])
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/grigorian/Hydro/sauvc/src/sauvc_startup/launch/missions.launch"])
                rospy.loginfo('Starting launch mode---------------------------------- ' + requestedMode)
                launch.start()
                launched = True
                launchRequested = False

            else:
                if requestedMode.count(':') == 1:
                    mode = requestedMode[:requestedMode.index(':')]
                    gate_fsm_mode = requestedMode[requestedMode.index(':')+1:]
                    rospy.set_param('/gateFsmMode', gate_fsm_mode)

                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nvidia/AUV/src/auv_startup/launch/" +
                    #                                                  mode + ".launch"])
                    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/grigorian/Hydro/sauvc/src/sauvc_startup/launch/missions.launch"])
                    rospy.loginfo('Starting launch mode ' + mode)
                    rospy.loginfo('Gate FSM mode ' + gate_fsm_mode)
                    launch.start()
                    launched = True
                    launchRequested = False

                else:
                    mode = requestedMode[:requestedMode.index(':')]
                    gate_fsm_mode = requestedMode[requestedMode.index(':')+1:requestedMode.rfind(':')]
                    rospy.set_param('/gateFsmMode', gate_fsm_mode)

                    drums_enabled = requestedMode[requestedMode.rfind(':')+1:]
                    rospy.set_param('/drumsEnabled', drums_enabled)

                    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                    roslaunch.configure_logging(uuid)
                    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nvidia/AUV/src/auv_startup/launch/" +
                    #                                                  mode + ".launch"])
                    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/grigorian/Hydro/sauvc/src/sauvc_startup/launch/missions.launch"])
                    rospy.loginfo('Starting launch mode ' + mode)
                    rospy.loginfo('Gate FSM mode ' + str(gate_fsm_mode))
                    rospy.loginfo('Drums enabled ' + str(drums_enabled))
                    launch.start()
                    launched = True
                    launchRequested = False


        elif stopRequested:
            rospy.loginfo('Shutting down launch config...')
            launch.shutdown()
            launched = False
            stopRequested = False

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
