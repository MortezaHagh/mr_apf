#! /usr/bin/env python

from typing import List
import rospy
import actionlib
from apf.msg import ApfAction, ApfGoal
from create_model import RobotsData


class PlanningClients:
    robots: RobotsData
    clients: List[actionlib.SimpleActionClient]

    def __init__(self, robots: RobotsData):
        rospy.loginfo("[PlanningClients], Initializing Path Planning Clinets.")
        self.ids = robots.ids
        self.robots = robots
        self.clients = []

    def send_goals(self):
        for rid in self.ids:
            goal = ApfGoal()
            goal.xt = self.robots.xt[rid]
            goal.yt = self.robots.yt[rid]
            name = "/r"+str(rid)+"/apf_action"
            client = actionlib.SimpleActionClient(name, ApfAction)
            client.wait_for_server()
            client.send_goal(goal)
            self.clients.append(client)
        rospy.sleep(0.4)


# if __name__ == "__main__":
#     rospy.init_node("send_goals")
#     xy = [[10, 10], [4, 4], [4, 10], [10, 4]]
#     for rid in range(0, 4):
#         goal = ApfGoal()
#         goal.xt = xy[rid][0]
#         goal.yt = xy[rid][1]
#         name = "/r"+str(rid)+"/apf_action"
#         client = actionlib.SimpleActionClient(name, ApfAction)
#         client.wait_for_server()
#         client.send_goal(goal)
#         rospy.sleep(0.4)
