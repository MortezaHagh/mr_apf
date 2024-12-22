#! /usr/bin/env python

import rospy
import actionlib
from parameters import Params
from create_model import MRSModel
from robot_planner_base import RobotPlanner
from robot_planner_2d import Planner2D
from robot_planner_rt import PlannerRT
from mrapf_classes import PRobot, PlannerData
from apf.msg import ApfAction, ApfResult, ApfGoal


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
        self.name_s = params.ns

        # shutdown hook
        rospy.on_shutdown(self.shutdown)

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
            rospy.loginfo(f"[RobotPlanner, {self.name_s}]: Succeeded!")
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo(f'Failed {self.ac_name}')
            self._as.set_aborted(self.result)

    def shutdown(self):
        rospy.loginfo(f"[RobotPlanner, {self.name_s}]: shutting down ... ")
        # rospy.sleep(1)
