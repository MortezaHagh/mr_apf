#! /usr/bin/env python

# pyright: reportMissingImports=false

import rospy
import actionlib
from parameters import Params
from apf_motion import ApfMotion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from apf.msg import ApfAction, ApfResult, ApfFeedback


class InitRobotAcion(object):

    def __init__(self, model, robot, ax):

        # pre
        self.mp = 0

        # attributes
        self.ax = ax
        self.robot = robot

        # shared data
        self.model = model

        # setting - parameters
        self.name_s = '/r' + str(robot.id)
        action_params = Params(robot.id)
        action_params.set_name_space(self.name_s)
        self.action_params = action_params

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # action: /r#/apf_action ---------------------------------------
        self._result = ApfResult()
        self._feedback = ApfFeedback()
        self.action_name = action_params.action_name
        self._as = actionlib.SimpleActionServer(self.action_name, ApfAction, self.goal_callback, False)
        self._as.start()
        print(self.name_s + ": Robot Action Server (" + self.action_name + ") has started.")

    # ---------------------   goal_callback   --------------------- #

    def goal_callback(self, goal):

        print(self.name_s + ": Robot Action Server (" + self.action_name + ") is called.")

        # goal received
        self.robot.xt = goal.xt * self.action_params.path_unit
        self.robot.yt = goal.yt * self.action_params.path_unit

        # get robots initial odom - start state
        self.get_odom()

        # motion planning   # --------------------------------------------------------------
        self.mp = ApfMotion(self.model, self.robot, self.action_params)
        success = self.mp.is_reached

        # result
        if success:
            self._result.goal_node = self.robot.goal_node
            self._result.start_node = self.robot.start_node
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self.action_name)
            self._as.set_aborted(self._result)

    # ---------------------------- get_odom- shutdown -------------------------------- #

    def get_odom(self):
        topic_msg = None
        rospy.loginfo(self.action_name + ", getting topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(
                    self.action_params.lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(self.action_name + ", current topic is ready.")
            except:
                rospy.loginfo(self.action_name + ", current topic is not ready yet, retrying ...")
        odom = topic_msg
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        orientation = orientation[2]
        position = odom.pose.pose.position
        self.robot.xs = position.x
        self.robot.ys = position.y
        self.robot.heading = round(orientation, 2)


    def shutdown(self):
        self.ax.plot(self.mp.path_x, self.mp.path_y)
        print(self.action_name + ', shutting down')
        rospy.sleep(1)
