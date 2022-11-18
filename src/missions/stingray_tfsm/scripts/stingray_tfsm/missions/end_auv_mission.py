from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.auv_control import AUVControl
import rospy


class EndAUVMission(AUVMission):
    """ Submission for robot initialization """

    def __init__(self,
                 name: str,
                 auv: AUVControl,
                 ):
        """ Submission for robot initialization

        Args:
            name (str): mission name
        """

        super().__init__(name, auv)

    def setup_states(self):
        return []

    def setup_transitions(self):
        return []

    def end_auv(self):
        self.machine.auv.execute_stop_goal()
        self.machine.auv.execute_dive_goal({
            'depth': 0,
        })
        rospy.sleep(10)
        self.enable_stabilization()

    def setup_scene(self):
        return {
            self.machine.state_init: {
                'preps': self.end_auv,
                "args": (),
            },
        }

    def setup_events(self):
        pass
