#! /usr/bin/env python

import rospy
import actionlib
from apf.msg import ApfAction, ApfGoal


class PlanningClients:
    def __init__(self, robots):
        self.ids = robots.ids
        self.robots = robots
        self.clients = []

    def send_goals(self):
        for id in self.ids:
            goal = ApfGoal()
            goal.xt = self.robots.xt[id]
            goal.yt = self.robots.yt[id]
            name = "/r"+str(id)+"/apf_action"
            client = actionlib.SimpleActionClient(name, ApfAction)
            client.wait_for_server()
            client.send_goal(goal)
            self.clients.append(client)
        rospy.sleep(0.4)


if __name__ == "__main__":
    rospy.init_node("send_goals")
    xy = [[10, 10], [4, 4], [4, 10], [10, 4]]
    for id in range(0, 4):
        goal = ApfGoal()
        goal.xt = xy[id][0]
        goal.yt = xy[id][1]
        name = "/r"+str(id)+"/apf_action"
        client = actionlib.SimpleActionClient(name, ApfAction)
        client.wait_for_server()
        client.send_goal(goal)
        rospy.sleep(0.4)
