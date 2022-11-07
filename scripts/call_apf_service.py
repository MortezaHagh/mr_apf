#! /usr/bin/env python

import rospy
from apf.srv import InitRobot, InitRobotRequest

def call_apf_service(ids):
    rate = rospy.Rate(20)

    # call services
    srv_name = "/init_apf_srv"
    rospy.wait_for_service(srv_name)
    initial_robots = rospy.ServiceProxy(srv_name, InitRobot)

    for id in ids:
        req = InitRobotRequest()
        req.id = id
        req.name = str(id)
        initial_robots(req)



if __name__ == "__main__":
    # ros
    rospy.init_node("call_apf_service")
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