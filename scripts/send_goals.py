#! /usr/bin/env python

# pyright: reportMissingImports=false

import rospy
import actionlib
from apf.msg import InitRobotAction, InitRobotGoal

rospy.init_node("send_goals")

xy =[[10, 10], [4, 4], [4, 10], [10, 4]]

for id in range(1,5):
    goal = InitRobotGoal()
    goal.xt = xy[id-1][0]
    goal.yt = xy[id-1][1]
    name = "/r"+str(id)+"/apf_action"
    client = actionlib.SimpleActionClient(name, InitRobotAction)
    client.wait_for_server()
    client.send_goal(goal)
    rospy.sleep(0.4)
