#! /usr/bin/env python

import rospy
from parameters import Params
from create_model import MRSModel, Robot
from robot_planner_l1_base import RobotPlannerBase


class PlannerRT(RobotPlannerBase):
    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        RobotPlannerBase.__init__(self, model, robot, params)

    def go_to_goal(self):
        while (not self.mrapf.reached) and (not rospy.is_shutdown()):
            if self.abort:
                self.stop()
                return False

            if not self.data_received:
                rospy.loginfo(f"[planner_ros, {self.ns}]: waiting for fleet data...")
                self.rate.sleep()
                continue

            # Move the robot
            self.move()
            self.rate.sleep()

            #
            if self.mrapf.reached:
                rospy.loginfo(f"[planner_ros, {self.ns}]: robot reached goal.")
        self.stop()
        return True
