from stingray_tfsm.auv_mission import AUVMission
import rospy


class InitAUVMission(AUVMission):
    """ Submission for robot initialization """

    def __init__(self,
                 name: str,
                 depth_stabilization: bool = False,
                 pitch_stabilization: bool = False,
                 yaw_stabilization: bool = False,
                 lag_stabilization: bool = False,
                 reset_imu: bool = False,
                 ):
        """ Submission for robot initialization

        Args:
            name (str): mission name
            depth_stabilization (bool, optional): flag to enable depth stabilization. Defaults to False.
            pitch_stabilization (bool, optional): flag to enable pitch stabilization. Defaults to False.
            yaw_stabilization (bool, optional): flag to enable yaw stabilization. Defaults to False.
            lag_stabilization (bool, optional): flag to enable lag stabilization. Defaults to False.
            reset_imu (bool, optional): flag to enable IMU reset on start. Defaults to False.
        """
        self.depth_stabilization = depth_stabilization
        self.pitch_stabilization = pitch_stabilization
        self.yaw_stabilization = yaw_stabilization
        self.lag_stabilization = lag_stabilization
        self.reset_imu = reset_imu

        super().__init__(name)

    def setup_states(self):
        return []

    def setup_transitions(self):
        return []

    def initialize_auv(self):
        if self.reset_imu:
            rospy.loginfo('Reset IMU')
            self.enable_reset_imu()
        # get current yaw
        self.machine.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })
        self.enable_stabilization(
            self.depth_stabilization, self.pitch_stabilization, self.yaw_stabilization, self.lag_stabilization)

        self.machine.auv.execute_dive_goal({
            'depth': 1000,
        })

        # init indication
        self.machine.auv.execute_move_goal({
            'march': 0.0,
            'lag': 0.2,
            'yaw': 0,
            'wait': 0.1
        })
        self.machine.auv.execute_stop_goal()
        # rospy.loginfo('Sleep before missions')
        # rospy.sleep(5)

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.initialize_auv,
                "args": (),
            },
        }

    def setup_events(self):
        pass
