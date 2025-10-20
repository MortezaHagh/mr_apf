#! /usr/bin/env python

""" Planning Clients for MRAPF, sending goals to each robot action server """
from typing import List
import rospy
import actionlib
from apf.msg import ApfAction, ApfGoal
from create_model import RobotsData


class PlanningClients:

    def __init__(self, robots: RobotsData):
        rospy.loginfo("[PlanningClients], Initializing Path Planning Clinets.")
        self.rids = robots.rids
        self.robots: RobotsData = robots
        self.clients: List[actionlib.SimpleActionClient] = []

    def send_goals(self):
        for rid in self.rids:
            goal = ApfGoal()
            goal.xt = self.robots.xt[rid]
            goal.yt = self.robots.yt[rid]
            name = "/r"+str(rid)+"/apf_action"
            client = actionlib.SimpleActionClient(name, ApfAction)
            client.wait_for_server()
            client.send_goal(goal)
            self.clients.append(client)
        rospy.sleep(0.4)

    def stop_planners(self):
        for c in self.clients:
            c.cancel_all_goals()
        rospy.sleep(0.4)
