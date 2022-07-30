#! /usr/bin/env python3

import rospkg
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
        self.launch_path = rospkg.RosPack().get_path('sauvc_startup') + '/launch/sauvc_'
        self.message = None
        # self.previousMessage = None

    def barcode_callback(self, msg):
        # rospy.loginfo('barcode_callback')
        # if self.callbackRequested == True:
        #     return
        # else:
        #     self.callbackRequested = True
        # self.previousMessage = self.message
        self.message = msg.data.lower()

        # if not (message in ['demo', 'stop', 'qualification', 'simple', 'medium', 'medium_ha']):
        #     rospy.logerr('Unknown messages')
        #     return
        #
        # if message == 'stop':
        #     if not self.launched:
        #         rospy.logwarn('Not launched yet')
        #         return
        #     else:
        #         rospy.loginfo('Shutting down launch config...')
        #         self.launch.shutdown()
        #         self.launched = False
        #         return
        #
        # if self.launched:
        #     rospy.logwarn('Already launched')
        #     return
        #
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_path + message + ".launch"]) #TODO: установить унивирсальный путь
        # rospy.loginfo('Starting launch mode ' + message)
        # launch.start()
        # rospy.loginfo('start ' + message)
        #
        # self.launched = True
        # self.callbackRequested = False

        return
def main():
    rospy.init_node('qr_trigger')
    qr = QrTrigger()
    rospy.Subscriber('/barcode', std_msgs.msg.String, qr.barcode_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if not (qr.message in ['demo', 'stop', 'qualification', 'simple', 'medium', 'medium_ha']):
            rospy.logerr('Unknown messages')
            rate.sleep()
            continue

        if qr.message == 'stop':
            if not qr.launched:
                rospy.logwarn('Not launched yet')
                rate.sleep()
                continue
            else:
                rospy.loginfo('Shutting down launch config...')
                qr.launch.shutdown()
                qr.launched = False
                rate.sleep()
                continue

        if qr.launched:
            rospy.logwarn('Already launched')
            rate.sleep()
            continue

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [qr.launch_path + qr.message + ".launch"]) #TODO: установить унивирсальный путь
        rospy.loginfo('Starting launch mode ' + qr.message)
        launch.start()
        rospy.loginfo('start ' + qr.message)

        qr.launched = True
        qr.callbackRequested = False

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
