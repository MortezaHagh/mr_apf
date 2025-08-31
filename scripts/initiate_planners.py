#! /usr/bin/env python

""" Initiate Planners by calling central service for each robot """

import rospy
from create_model import MRSModel
from apf.srv import InitRobot, InitRobotRequest


def initiate_robots_planners(model: MRSModel):
    """ Initializing Planners by calling central service for each robot.
        Simulating each robot connecting to the central service.
    """
    rospy.loginfo("Initializing Planners by calling central service for each robot.")
    rate = rospy.Rate(20)
    srv_name = "/central_mrapf_srv"
    rospy.wait_for_service(srv_name)
    initial_robots = rospy.ServiceProxy(srv_name, InitRobot)
    for r in model.robots:
        req = InitRobotRequest()
        req.rid = r.rid
        req.xs = r.xs
        req.ys = r.ys
        req.xt = r.xt
        req.yt = r.yt
        req.theta = r.heading
        req.name = str(r.rid)
        initial_robots(req)
        rate.sleep()
