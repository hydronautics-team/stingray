#!/usr/bin/env python

import rospy
import actionlib
import src.stingray_movement_msgs.msg as msg


class AUV:
    """All actions are unlocked and are supposed to be used asynchronously"""

    def __init__(self):
        rospy.loginfo("Initializing action servers")

        rospy.loginfo("Plane move init")
        self.MoveClient = actionlib.SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)
        rospy.loginfo("Waiting for server")
        self.MoveClient.wait_for_server()
        rospy.loginfo("Success")

        rospy.loginfo("Rotation init")
        self.RotateClient = actionlib.SimpleActionClient('stingray_action_rotate', msg.RotateAction)
        rospy.loginfo("Waiting for server")
        self.absolute_angle = 0
        self.RotateClient.wait_for_server()
        rospy.loginfo("Success")

        rospy.loginfo("Vertical control init")
        self.DiveClient = actionlib.SimpleActionClient('stingray_action_dive', msg.DiveAction)
        rospy.loginfo("Waiting for server")
        self.DiveClient.wait_for_server()
        rospy.loginfo("Success")

    @staticmethod
    def callback_active():
        rospy.loginfo("Action server is processing the goal")

    @staticmethod
    def callback_done(state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    @staticmethod
    def callback_feedback(feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    def rotate(self, angle=0):
        self.absolute_angle += angle
        rospy.loginfo("Absolute angle is %{} now".format(self.absolute_angle))

        if angle != 0:
            goal = msg.RotateGoal(yaw=self.absolute_angle)
            rospy.loginfo("Sending goal")
            self.RotateClient.send_goal(goal,
                                        active_cb=self.callback_active,
                                        feedback_cb=self.callback_feedback,
                                        done_cb=self.callback_done)

    def forward_locked(self, duration=0, velocity=0.0):
        """angle in degrees, duration in ms, velocity is relative(from 0 to 1)"""
        if duration != 0 and velocity != 0:
            goal = msg.LinearMoveGoal(direction=1, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            self.MoveClient.send_goal(goal,
                                      active_cb=self.callback_active,
                                      feedback_cb=self.callback_feedback,
                                      done_cb=self.callback_done)

            rospy.loginfo("Waiting for result")
            self.MoveClient.wait_for_result()
            rospy.loginfo("Forward/backwards movement done")

    def backwards_locked(self, duration=0, velocity=0.0):
        self.forward_locked(duration, -velocity)

    def lag_right(self, duration=0, velocity=0.0):
        if duration != 0 and velocity != 0:
            goal = msg.LinearMoveGoal(direction=3, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            self.MoveClient.send_goal(goal,
                                      active_cb=self.callback_active,
                                      feedback_cb=self.callback_feedback,
                                      done_cb=self.callback_done)

    def lag_left(self, duration=0, velocity=0.0):
        self.lag_right(duration, -velocity)

    def dive(self, depth=100):
        goal = msg.DiveGoal(depth=depth)
        print("Sending diving goal")
        self.DiveClient.send_goal(goal,
                                  active_cb=self.callback_active,
                                  feedback_cb=self.callback_feedback,
                                  done_cb=self.callback_done)

    def execute_pattern(self, path, size=1000, speed=0.35, clockwise=True, straight=True):
        """
        :param path: string, path to pattern script
        :param size: int, base work time for engines in ms, used to scale pattern
        :param speed:  float from 0 to 1. Ratio of current speed to max
        :param clockwise: If False, executes reversed/mirrored pattern actions
        :param straight:  If False, executes pattern script from last action to first
        :return:
        """
        scriptf = open(path)
        script = scriptf.readlines()
        scriptf.close()

        # here script name should be read
        if not straight:
            script = script[::-1]

        if clockwise:
            clockwise = 1
        else:
            clockwise = -1

        for action in script:
            if not action:
                continue
            elif action[0] == '#':
                continue

            action = action.split()

            if action[0] == "mvf":
                if action[1].isdigit():
                    self.forward_locked(int(action[1])*size, speed*clockwise)

        # TODO create movement patterns config or smth like that
        pass

    def circle(self, radius, speed=0.35, clockwise=True):
        if clockwise:
            clockwise = 1
        else:
            clockwise = -1

        self.rotate(-clockwise*60)
        self.forward_locked(radius, speed)
        for i in range(5):
            self.rotate(clockwise*60)
            self.forward_locked(radius, speed)
        self.rotate(clockwise*120)

