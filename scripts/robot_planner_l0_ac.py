#! /usr/bin/env python

from copy import deepcopy
import rospy
import actionlib
from parameters import Params
from mrapf_classes import PlannerData
from robot_planner_l2_2d import Planner2D
from robot_planner_l2_rt import PlannerRT
from create_model import MRSModel, Robot
from robot_planner_l1_base import RobotPlannerBase
from apf.msg import ApfAction, ApfResult, ApfGoal


class RobotPlannerAc:
    model: MRSModel
    robot: Robot
    params: Params
    planner_data = PlannerData

    def __init__(self, model, robot: Robot):

        #
        self.robot = robot
        self.model = model
        self.planner_data = None

        # setting - parameters
        ns = "r" + str(robot.rid)
        params = deepcopy(self.model.params)
        params.set_rid(robot.rid)
        params.set_ns(ns)
        self.params = params
        self.ns = params.ns

        # planner
        self.planner: RobotPlannerBase = None

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # action: /r#/apf_action ===============================================
        self.result = ApfResult()
        self.ac_name = params.ac_name
        self._as = actionlib.SimpleActionServer(self.ac_name, ApfAction, self.goal_callback, False)
        self._as.start()
        rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: Robot Action Server [{self.ac_name}] has started.")

    def goal_callback(self, goal: ApfGoal):

        rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: Robot Action Server [{self.ac_name}] is called.")

        # goal received
        self.robot.xt = goal.xt
        self.robot.yt = goal.yt

        # motion planning   # ==================================================
        success = False
        self.planner: RobotPlannerBase = None
        if self.params.simD == "2D":
            self.planner = Planner2D(self.model, self.robot, self.params)
        else:  # "3D"
            self.planner = PlannerRT(self.model, self.robot, self.params)
        self.planner.start_planner()
        success = self.planner.is_reached
        self.planner_data = self.planner.pd

        # result
        if success:
            # ac result
            self.result.result = True
            self.result.path_x = self.planner.pd.x
            self.result.path_y = self.planner.pd.y
            rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: Succeeded!")
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: Failed!")
            self._as.set_aborted(self.result)

    def stop_planner(self):
        rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: stopping planner ... ")
        self.planner.stop_planner()

    def shutdown(self):
        rospy.loginfo(f"[RobotPlannerAC, {self.ns}]: shutting down ... ")
        self.planner.stop_planner()
