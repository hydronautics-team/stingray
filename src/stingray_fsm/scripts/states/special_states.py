import rospy
import smach
import smach_ros

from std_srvs.srv import SetBool
from stingray_drivers_msgs.srv import SetStabilization


class InitializationState(smach.State):
    def __init__(self, delay_after_init):
        smach.State.__init__(self, outcomes=["INIT_OK"])
        self.delayAfterInit_ = delay_after_init

    def execute(self, userdata):
        rospy.loginfo("Resetting IMU...")
        service = rospy.ServiceProxy("/stingray/services/control/set_imu_enabled", SetBool)
        response = service(True)
        rospy.loginfo("IMU has been reset!")
        rospy.sleep(self.delayAfterInit_ / 1000.0)
        service = rospy.ServiceProxy("/stingray/services/control/set_stabilization", SetStabilization)
        response = service(True, True, False)
        rospy.loginfo("Stabilization enabled")
        return "INIT_OK"


