from stingray_tfsm.auv_mission import AUVMission
import rospy


class EndAUVMission(AUVMission):
    """ Submission for robot initialization """

    def __init__(self,
                 name: str,
                 ):
        """ Submission for robot initialization

        Args:
            name (str): mission name
        """

        super().__init__(name)

    def setup_states(self):
        return []

    def setup_transitions(self):
        return []

    def end_auv(self):
        self.machine.auv.execute_stop_goal()
        self.machine.auv.execute_dive_goal({
            'depth': 100,
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
