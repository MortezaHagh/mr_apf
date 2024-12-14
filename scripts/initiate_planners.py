#! /usr/bin/env python

import rospy
from apf.srv import InitRobot, InitRobotRequest


def initiate_robots_planners(ids):
    """ call central service to create Planner for robot 
    """
    rate = rospy.Rate(20)
    srv_name = "/central_mrapf_srv"
    rospy.wait_for_service(srv_name)
    initial_robots = rospy.ServiceProxy(srv_name, InitRobot)

    for id in ids:
        req = InitRobotRequest()
        req.id = id
        req.name = str(id)
        initial_robots(req)
        rate.sleep()


if __name__ == "__main__":
    # ros
    rospy.init_node("initiate_robots_planners")
    rate = rospy.Rate(20)
    srv_name = "/central_mrapf_srv"
    rospy.wait_for_service(srv_name)
    initial_robots = rospy.ServiceProxy(srv_name, InitRobot)
    ids = [1, 2, 3, 4]
    for id in ids:
        req = InitRobotRequest()
        req.id = id
        req.name = str(id)
        initial_robots(req)
        rate.sleep()
