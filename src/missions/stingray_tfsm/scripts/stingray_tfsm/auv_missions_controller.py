
from abc import abstractmethod
from stingray_tfsm.core.pure_missions_controller import PureMissionsController
from stingray_tfsm.missions.init_auv_mission import InitAUVMission
from stingray_tfsm.missions.end_auv_mission import EndAUVMission
from stingray_tfsm.auv_control import AUVControl


class AUVMissionsController(PureMissionsController):
    """ Class for controlling AUV missions """

    @abstractmethod
    def __init__(self,
                 camera: str,
                 auv: AUVControl,
                 depth_stabilization: bool = False,
                 pitch_stabilization: bool = False,
                 yaw_stabilization: bool = False,
                 lag_stabilization: bool = False,
                 reset_imu: bool = False,
                 ):
        """ Class for controlling AUV missions.
        """
        self.camera = camera
        self.depth_stabilization = depth_stabilization
        self.pitch_stabilization = pitch_stabilization
        self.yaw_stabilization = yaw_stabilization
        self.lag_stabilization = lag_stabilization
        self.reset_imu = reset_imu
        self.auv = auv

        super().__init__()

    def add_init_mission(self):
        init_mission = InitAUVMission(
            InitAUVMission.__name__,
            self.camera,
            self.auv,
            self.depth_stabilization,
            self.pitch_stabilization,
            self.yaw_stabilization,
            self.lag_stabilization,
            self.reset_imu,
        )
        self.add_mission(init_mission)
    
    def add_end_mission(self):
        end_mission = EndAUVMission(
            EndAUVMission.__name__,
            self.auv
        )
        self.add_mission(end_mission)
