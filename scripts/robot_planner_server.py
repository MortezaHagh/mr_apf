#! /usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from parameters import Params
from create_model import MRSModel
from robot_planner import RobotPlanner
from robot_planner_2d import Planner2D
from robot_planner_ros import PlannerROS
from mrapf_classes import PRobot, PlannerData
from apf.msg import ApfAction, ApfResult, ApfGoal
from apf.srv import SharePoses2, SharePoses2Request


class RobotPlannerAc:
    model: MRSModel
    robot: PRobot
    params: Params
    planner_data = PlannerData

    def __init__(self, model, robot: PRobot, params: Params):

        #
        self.robot = robot
        self.model = model
        self.planner_data = None

        # setting - parameters
        self.params = params
        self.name_s = params.name_space

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # pose service client
        rospy.wait_for_service(params.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(params.pose_srv_name, SharePoses2)

        # action: /r#/apf_action ===============================================
        self.result = ApfResult()
        self.ac_name = params.ac_name
        self._as = actionlib.SimpleActionServer(self.ac_name, ApfAction, self.goal_callback, False)
        self._as.start()
        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Robot Action Server [{self.ac_name}] has started.")

    def goal_callback(self, goal: ApfGoal):

        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Robot Action Server [{self.ac_name}] is called.")

        # goal received
        self.robot.xt = goal.xt * self.params.path_unit
        self.robot.yt = goal.yt * self.params.path_unit

        # get robots initial odom - start state
        self.get_odom()

        # update target coordinates in pose service
        req = SharePoses2Request()
        req.id = self.robot.id
        req.xt = self.robot.xt
        req.yt = self.robot.yt
        req.update = True
        self.pose_client(req)

        # motion planning   # ==================================================
        success = False
        planner: RobotPlanner = None
        if self.params.sim == "2D":
            planner = Planner2D(self.model, self.robot, self.params)
        else:  # "3D"
            planner = PlannerROS(self.model, self.robot, self.params)
        planner.start()
        success = planner.is_reached

        # result
        if success:
            self.planner_data = planner.pd
            # ac result
            self.result.result = True
            self.result.path_x = planner.pd.x
            self.result.path_y = planner.pd.y
            rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Succeeded!")
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo(f'Failed {self.ac_name}')
            self._as.set_aborted(self.result)

    def get_odom(self):
        topic_msg: Odometry = None
        rospy.loginfo(self.ac_name + ", getting topic ...")
        while topic_msg is None:
            try:
                topic_msg = rospy.wait_for_message(self.params.lis_topic, Odometry, timeout=3.0)
                rospy.logdebug(self.ac_name + ", current topic is ready.")
            except rospy.ROSInterruptException as e:
                rospy.logerr(f"ROS node was interrupted: {e}")
            except rospy.ROSException as e:
                rospy.logerr(f"Timeout while waiting for message: {e}")
                rospy.loginfo(f"{self.ac_name}, current topic is not ready yet, retrying ...")
        odom = topic_msg
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        orientation = orientation[2]
        position = odom.pose.pose.position
        self.robot.xs = position.x
        self.robot.ys = position.y
        self.robot.heading = round(orientation, 2)

    def shutdown(self):
        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: shutting down ... ")
        # rospy.sleep(1)
