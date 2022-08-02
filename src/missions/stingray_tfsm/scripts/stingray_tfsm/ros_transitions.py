from stingray_tfsm.pure_transitions import FSM_Simple
from actionlib import SimpleActionClient
import stingray_movement_msgs.msg as msg
from std_msgs.msg import Int32
import rospy


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" %
                  (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


class AUVStateMachine(FSM_Simple):
    def __init__(self, states: tuple, transitions: list = None, scene: dict = None, path=None, verbose=True):
        """
        The __init__ function is called when an instance of the class is created.
        :param self: Reference the object itself
        :param states:tuple: Define the states of the state machine
        :param transitions:list=None: Pass a list of transitions to the super class
        :param scene:dict=None: Pass a dictionary of objects in the scene
        :param path=None: Specify the path to a file that contains the states and transitions
        :param verbose=True: Print out the state of the robot as it moves through its states
        :return: The object of the class
        :doc-author: Trelent
        """
        super().__init__(states, transitions, path, verbose)
        self.LinearMoveClient = SimpleActionClient(self.ros_config["actions"]["movement"]["linear"], msg.LinearMoveAction)
        self.scene = scene
        self.absolute_angle = 0
        self._get_yaw()
        self.RotateClient = SimpleActionClient(self.ros_config["actions"]["movement"]["rotate"], msg.RotateAction)
        self.DiveClient = SimpleActionClient(self.ros_config["actions"]["movement"]["dive"], msg.DiveAction)

    def yaw_topic_callback(self, msg):
        """
        The yaw_topic_callback function is a callback function that is called every time the yaw_topic publishes a message.
        The yaw_topic is the topic that publishes the current angle of rotation of the drone.
        This function sets self.absolute_angle to this value, which will be used in other functions.

        :param self: Access the variables and methods inside a class
        :param msg: Store the data from the topic
        :return: None
        :doc-author: Trelent
        """
        self.absolute_angle = msg.data
        if self.verbose:
            rospy.loginfo(
                f"Absolute angle got from machine is {msg.data}; It is set on higher level")

    def dummy(self, scene):
        """
        The dummy function is used to test the docstring parsing code.
        :param self: Refer to the instance of the class
        :param scene: Access the scene object
        :return: None
        :doc-author: Trelent
        """
        rospy.loginfo(scene['message'])
        rospy.sleep(10)

    def execute_move_goal(self, scene):
        """
        The execute_move_goal function sends a goal to the LinearMoveClient action server.
        The goal is defined by the scene dictionary, which contains direction, velocity and duration.
        The function waits for the result of the action execution and returns it.

        :param self: Access the class attributes and methods
        :param scene: Pass the scene information to the execute_move_goal function
        :return: :
        :doc-author: Trelent
        """
        goal = msg.LinearMoveGoal(
            scene['direction'], scene['velocity'], scene['duration'])
        self.LinearMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.LinearMoveClient.wait_for_result(
            timeout=rospy.Duration(secs=scene['duration'] // 1000 + 1))
        rospy.loginfo('Result got')

    def execute_dive_goal(self, scene):
        """
        The execute_dive_goal function sends a goal to the Dive action server.
        It waits for the action server to finish performing the task, then it returns whether or not
        the task was successful. The function also sets up callbacks that allow us to print feedback
        and determine when the task is complete.

        :param self: Access the variables and methods of the class
        :param scene: Pass the scene information to the dive action server
        :return: The result of the action
        :doc-author: Trelent
        """
        goal = msg.DiveClientGoal(scene['depth'])
        self.DiveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                  active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.DiveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        rospy.loginfo('Result got')

    def _get_yaw(self):
        """
        The _get_yaw function forces the update of absolute angle using yaw_topic_callback

        :param self: Reference the class object within the function
        :return: None
        :doc-author: Trelent
        """
        self._topic_sub = rospy.Subscriber(self.ros_config["topics"]["yaw"],
                                           Int32,
                                           callback=self.yaw_topic_callback,
                                           queue_size=10)
        self._topic_sub.unregister()

    def execute_rotate_goal(self, scene):
        """
        The execute_rotate_goal function rotates the robot by a specified angle.
        :param self: Access the attributes and methods of the class in python
        :param scene: Pass the scene information to the rotate goal function
        :return: The result of the action server
        :doc-author: Trelent
        """
        angle = scene['angle']
        self._get_yaw()
        self.absolute_angle += angle

        if self.absolute_angle > 360:
            self.absolute_angle -= 360
        elif self.absolute_angle < -360:
            self.absolute_angle += 360

        rospy.loginfo("Absolute angle is {} now".format(self.absolute_angle))

        if angle != 0:
            goal = msg.RotateGoal(yaw=self.absolute_angle)
            self.RotateClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
            if self.verbose:
                rospy.sleep(0.1)
            rospy.loginfo('Goal sent')
            self.RotateClient.wait_for_result(timeout=rospy.Duration(secs=5))
            rospy.loginfo('Result got')
        else:
            rospy.loginfo('Rotating is not required to achieve this angle')

    def next_step(self):
        """
        The next_step function is the main function of the ros machine. It is called
        every time a new state is entered and it will execute all actions associated with
        that state. The next_step function also triggers the calculation of conditions,
        and chooses the next transition basing on the calculations
        :param self: Access the class attributes and methods
        :return: None
        :doc-author: Trelent
        """
        state_keyword = self.state.split('_')[0]
        scene = self.scene[self.state]

        if self.verbose:
            rospy.loginfo(
                f"DEBUG: Current state of ros machine is {self.state}")

        if rospy.is_shutdown():
            self.set_state('aborted')

        elif state_keyword == 'init':
            rospy.sleep(scene['time'])

        elif state_keyword == 'dummy':
            self.dummy(scene)

        elif state_keyword == 'move':
            self.execute_move_goal(scene)

        elif state_keyword == 'rotate':
            self.execute_rotate_goal(scene)

        elif state_keyword == 'dive':
            self.execute_dive_goal(scene)

        elif state_keyword == 'condition':
            if 'subFSM' in scene:
                if scene['subFSM']:
                    scene['condition'].set_state('init')
                    decision = scene['condition'].run(scene['args'])
                else:
                    decision = scene['condition'](scene['args'])
            else:
                decision = scene['condition'](scene['args'])
            if decision:
                if self.verbose:
                    rospy.loginfo("DEBUG: Current condition results True")
                self.trigger('condition_s')
            else:
                if self.verbose:
                    rospy.loginfo("DEBUG: Current condition results False")
                self.trigger('condition_f')
            return
        elif state_keyword == 'done':
            exit()
        self.trigger(self.fsm.get_triggers(self.state)[0])
