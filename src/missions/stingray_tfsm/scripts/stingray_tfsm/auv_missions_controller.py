
from abc import abstractmethod
from stingray_tfsm.core.pure_missions_controller import PureMissionsController
from stingray_tfsm.missions.init_auv_mission import InitAUVMission


class AUVMissionsController(PureMissionsController):
    """ Class for controlling AUV missions """

    @abstractmethod
    def __init__(self,
                 depth_stabilization: bool = False,
                 yaw_stabilization: bool = False,
                 lag_stabilization: bool = False,
                 reset_imu: bool = False,
                 ):
        """ Class for controlling AUV missions.
        """
        self.depth_stabilization = depth_stabilization
        self.yaw_stabilization = yaw_stabilization
        self.lag_stabilization = lag_stabilization
        self.reset_imu = reset_imu

        super().__init__()

    def setup_missions(self):
        init_mission = InitAUVMission(
            InitAUVMission.__name__,
            self.depth_stabilization,
            self.yaw_stabilization,
            self.lag_stabilization,
            self.reset_imu,
        )
        self.add_mission(init_mission)
