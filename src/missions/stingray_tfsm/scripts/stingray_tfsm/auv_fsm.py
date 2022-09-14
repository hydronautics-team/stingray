from stingray_tfsm.core.pure_fsm import PureStateMachine
import stingray_movement_msgs.msg as msg
from stingray_tfsm.auv_control import AUVControl
from std_msgs.msg import Int32
import rospy


class AUVStateMachine(PureStateMachine):
    def __init__(self, name: str, states: tuple = (), transitions: list = [], scene: dict = {}, path=None, verbose=False):
        """ State machine for AUV

        :param name:str=(): Define the name of the machine
        :param states:tuple: Define the states of the state machine
        :param transitions:list=None: Pass a list of transitions to the super class
        :param scene:dict=None: Pass a dictionary of objects in the scene
        :param path=None: Specify the path to a file that contains the states and transitions
        :param verbose=True: Print out the state of the robot as it moves through its states

        """
        self.auv = AUVControl()

        super().__init__(name, states, transitions, scene, path)

    def next_step(self):
        """
        The next_step function is the main function of the ros machine. It is called
        every time a new state is entered and it will execute all actions associated with
        that state. The next_step function also triggers the calculation of conditions,
        and chooses the next transition basing on the calculations
        :param self: Access the class attributes and methods
        :return: None

        """
        state = self.state
        rospy.loginfo(f"FSM: {self.name}\tSTATE: {self.state}")
        
        if self.name.upper() in state:
            state = state.replace(self.name.upper() + "_", "")

        state_keyword = state.split('_')[0].lower()

        scene = self.scene[self.state]

        next_trigger = self.machine.get_triggers(self.state)[0]

        if rospy.is_shutdown():
            self.set_state(self.state_aborted)
            self.auv.execute_stop_goal()

        elif state_keyword == 'custom':
            if 'subFSM' in scene:
                if scene['subFSM']:
                    scene['custom'].set_init_state()
                    scene['custom'].run(*scene['args'])
                else:
                    scene['custom'](*scene['args'])
            else:
                scene['custom'](*scene['args'])

        elif state_keyword == 'init':
            if 'time' in scene:
                rospy.sleep(scene['time'])
            if 'preps' in scene:
                scene['preps'](*scene['args'])

        elif state_keyword == 'move':
            self.auv.execute_move_goal(scene)
        
        elif state_keyword == 'dive':
            self.auv.execute_dive_goal(scene)

        elif state_keyword == 'condition':
            if 'subFSM' in scene:
                if scene['subFSM']:
                    scene['condition'].set_init_state()
                    decision = scene['condition'].run(*scene['args'])
                else:
                    decision = scene['condition'](*scene['args'])
            else:
                decision = scene['condition'](*scene['args'])
            if decision:
                if self.verbose:
                    rospy.loginfo("DEBUG: Current condition results True")
                next_trigger = 'condition_s'
            else:
                if self.verbose:
                    rospy.loginfo("DEBUG: Current condition results False")
                next_trigger = 'condition_f'
        elif state_keyword == self.state_end:
            exit()

        rospy.loginfo(f"FSM: {self.name}\tTRANSITION: {next_trigger}")
        self.trigger(next_trigger)
