#! /usr/bin/env python3

import rospy
import roslaunch
import std_msgs.msg

class QrTrigger:
    def __init__(self):
        self.launched = False
        self.callbackRequested = False
        self.requestedMode = None
        self.launch = None
        rospy.loginfo('__init__ QrTrigger')
        rospy.Subscriber('/barcode', std_msgs.msg.String, self.barcode_callback)

    def barcode_callback(self, msg):
        rospy.loginfo('barcode_callback')
        if self.callbackRequested == True:
            return
        else:
            self.callbackRequested == True

        message = msg.data.lower()

        if not (message in ['demo', 'stop', 'qualification', 'simple', 'medium', 'medium_ha']):
            rospy.logerr('Unknown messages')
            return

        if message == 'stop':
            if not self.launched:
                rospy.logwarn('Not launched yet')
                return
            else:
                rospy.loginfo('Shutting down launch config...')
                self.launch.shutdown()
                self.launched = False
                return

        if self.launched:
            rospy.logwarn('Already launched')
            return

        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/grigorian/Hydro/sauvc/src/sauvc_startup/launch/sauvc_" +
        #                                                  message + ".launch"]) #TODO: установить унивирсальный путь
        # rospy.loginfo('Starting launch mode ' + message)
        # launch.start()
        rospy.loginfo('start ' + message)

        self.launched == True
        self.callbackRequested == False

        return

if __name__ == '__main__':
    try:
        rospy.init_node('qr_trigger')
        qr = QrTrigger()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
