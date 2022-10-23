#! /usr/bin/env python

import rospy
from apf.srv import InitRobot, InitRobotRequest
# ros
rospy.init_node("main_node")
rate = rospy.Rate(20)

# call services
srv_name = "/init_apf_srv"
rospy.wait_for_service(srv_name)
initial_robots = rospy.ServiceProxy(srv_name, InitRobot)

ids = [1,2,3,4]

for id in ids:
    req = InitRobotRequest()
    req.id = id
    req.name = str(id)
    initial_robots(req)