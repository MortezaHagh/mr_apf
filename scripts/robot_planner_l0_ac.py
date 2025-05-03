#! /usr/bin/env python

import rospy
import actionlib
from parameters import Params
from mrapf_classes import PlannerData
from robot_planner_l2_2d import Planner2D
from robot_planner_l2_rt import PlannerRT
from create_model import MRSModel, Robot
from robot_planner_l1_base import RobotPlanner
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
        params = Params(robot.rid)
        params.set_ns(ns)
        self.params = params
        self.ns = params.ns

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

        # action: /r#/apf_action ===============================================
        self.result = ApfResult()
        self.ac_name = params.ac_name
        self._as = actionlib.SimpleActionServer(self.ac_name, ApfAction, self.goal_callback, False)
        self._as.start()
        rospy.loginfo(f"[RobotPlanner, {self.ns}]: Robot Action Server [{self.ac_name}] has started.")

    def goal_callback(self, goal: ApfGoal):

        rospy.loginfo(f"[RobotPlanner, {self.ns}]: Robot Action Server [{self.ac_name}] is called.")

        # goal received
        self.robot.xt = goal.xt * self.params.path_unit
        self.robot.yt = goal.yt * self.params.path_unit

        # motion planning   # ==================================================
        success = False
        planner: RobotPlanner = None
        if self.params.sim == "2D":
            planner = Planner2D(self.model, self.robot, self.params)
        else:  # "3D"
            planner = PlannerRT(self.model, self.robot, self.params)
        planner.start()
        success = planner.is_reached

        # result
        if success:
            self.planner_data = planner.pd
            # ac result
            self.result.result = True
            self.result.path_x = planner.pd.x
            self.result.path_y = planner.pd.y
            rospy.loginfo(f"[RobotPlanner, {self.ns}]: Succeeded!")
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo(f'Failed {self.ac_name}')
            self._as.set_aborted(self.result)

    def shutdown(self):
        rospy.loginfo(f"[RobotPlanner, {self.ns}]: shutting down ... ")
        # rospy.sleep(1)
