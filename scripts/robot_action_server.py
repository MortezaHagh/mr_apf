#! /usr/bin/env python

# pyright: reportMissingImports=false

import rospy
import actionlib
from apf_motion_1 import ApfMotion #apf_motion_1
from nav_msgs.msg import Odometry
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion
from apf.msg import ApfAction, ApfResult, ApfFeedback


class InitRobotAcion(object):

    def __init__(self, model, robot, action_params):

        # times
        self.time = [0, 0, 0]

        # pre
        self.mp = 0

        # attributes
        self.robot = robot

        # shared data
        self.model = model

        # setting - parameters
        self.action_params = action_params
        self.name_s = action_params.name_space

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # pose service client
        rospy.wait_for_service(action_params.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(action_params.pose_srv_name, SharePoses2)

        # action: /r#/apf_action ---------------------------------------
        self.result = ApfResult()
        self.feedback = ApfFeedback()
        self.ac_name = action_params.ac_name
        self._as = actionlib.SimpleActionServer(self.ac_name, ApfAction, self.goal_callback, False)
        self._as.start()
        print(self.name_s + ": Robot Action Server (" + self.ac_name + ") has started.")

    # ---------------------   goal_callback   --------------------- #

    def goal_callback(self, goal):

        print(self.name_s + ": Robot Action Server (" + self.ac_name + ") is called.")

        # goal received
        self.robot.xt = goal.xt * self.action_params.path_unit
        self.robot.yt = goal.yt * self.action_params.path_unit

        # get robots initial odom - start state
        self.get_odom()

        # update target coordinates in pose service
        req = SharePoses2Request()
        req.ind = self.robot.id
        req.xt = self.robot.xt
        req.yt = self.robot.yt
        req.update = True
        self.pose_client(req)

        # start time
        self.time[0] = rospy.get_time()

        # motion planning   # --------------------------------------------------------------
        self.success = False
        self.mp = ApfMotion(self.model, self.robot, self.action_params)
        self.success = self.mp.is_reached

        # time
        self.time[1] = rospy.get_time()
        self.time[2] = round(self.time[1] - self.time[0], 2)

        # result
        if self.success:
            self.result.result = True
            self.result.path_x = self.mp.path_x
            self.result.path_y = self.mp.path_y
            rospy.loginfo('%s: Succeeded' % self.ac_name)
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo('%s: Failed' % self.ac_name)
            self._as.set_aborted(self.result)

    # ---------------------------- get_odom- shutdown -------------------------------- #

    def get_odom(self):
        topic_msg = None
        rospy.loginfo(self.ac_name + ", getting topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(self.action_params.lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(self.ac_name + ", current topic is ready.")
            except:
                rospy.loginfo(self.ac_name + ", current topic is not ready yet, retrying ...")
        odom = topic_msg
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        orientation = orientation[2]
        position = odom.pose.pose.position
        self.robot.xs = position.x
        self.robot.ys = position.y
        self.robot.heading = round(orientation, 2)


    def shutdown(self):
        print(self.ac_name + ', shutting down')
        # rospy.sleep(1)
