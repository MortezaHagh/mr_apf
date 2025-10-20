#! /usr/bin/env python

""" Planning Clients for MRAPF, sending goals to each robot action server """
from typing import List
import rospy
import actionlib
from apf.msg import ApfAction, ApfGoal
from create_model import Robot


class PlanningClients:

    def __init__(self, robots: List[Robot]):
        rospy.loginfo("[PlanningClients], Initializing Path Planning Clients.")
        self.rids = [robot.rid for robot in robots]
        self.robots: List[Robot] = robots
        self.clients: List[actionlib.SimpleActionClient] = []

    def send_goals(self):
        for robot in self.robots:
            goal = ApfGoal()
            goal.xt = robot.xt
            goal.yt = robot.yt
            name = "/r"+str(robot.rid)+"/apf_action"
            client = actionlib.SimpleActionClient(name, ApfAction)
            client.wait_for_server()
            client.send_goal(goal)
            self.clients.append(client)
        rospy.sleep(0.4)

    def stop_planners(self):
        for c in self.clients:
            c.cancel_all_goals()
        rospy.sleep(0.4)
