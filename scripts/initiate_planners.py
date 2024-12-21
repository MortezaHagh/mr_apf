#! /usr/bin/env python

from typing import List
import rospy
from apf.srv import InitRobot, InitRobotRequest


def initiate_robots_planners(ids: List[int]):
    """ Initializing Planners by calling central service for each robot.
        Simulating each robot connecting to the central service.
    """
    rospy.loginfo("Initializing Planners by calling central service for each robot.")
    rate = rospy.Rate(20)
    srv_name = "/central_mrapf_srv"
    rospy.wait_for_service(srv_name)
    initial_robots = rospy.ServiceProxy(srv_name, InitRobot)
    for rid in ids:
        req = InitRobotRequest()
        req.rid = rid
        req.name = str(rid)
        initial_robots(req)
        rate.sleep()
