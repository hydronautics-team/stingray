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
    if not (message in ['demo', 'stop', 'qualification', 'simple', 'medium', 'medium_ha']):
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
    rospy.Service('global_fsm_finished', std_srvs.srv.Trigger, stop_notification_callback) # я не знаю что это за строчка

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if launchRequested:
            rospy.loginfo('while not rospy.is_shutdown ' + requestedMode)
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/grigorian/Hydro/sauvc/src/sauvc_startup/launch/sauvc_" +
                                                             requestedMode + ".launch"]) #TODO: установить унивирсальный путь
            rospy.loginfo('Starting launch mode ' + requestedMode)
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
