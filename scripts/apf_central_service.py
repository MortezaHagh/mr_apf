#! /usr/bin/env python

# pyright: reportMissingImports=false

import rospy
import numpy as np
from robot_action_server import InitRobotAcion
from apf.srv import InitRobot, InitRobotResponse


class Robot(object):
    def __init__(self, xs=0, ys=0, id=0, name="r", heading=0, xt=0, yt=0):
        self.id = id
        self.xs = xs
        self.ys = ys
        self.xt = xt
        self.yt = yt
        self.name = name
        self.heading = heading

    # -------------------------------- InitRobotService -------------------------- #

class InitRobotService(object):
    def __init__(self, model, init_srv_name, ax):

        # data
        self.ax = ax
        self.model = model

        # all robots
        self.robots = []
        self.robots_id = []
        self.robot_count = 0
        self.ac_services = []

        # init_robot_srv Service Server
        init_apf__srv_name = init_srv_name
        self.robot_srv = rospy.Service(init_apf__srv_name, InitRobot, self.apf_srv_callback)
        print("Central Service Server (init_apf_srv) is created. ")

    # -------------------------------- robot_srv_callback -------------------------- #

    def apf_srv_callback(self, req):

        print("Central Service (init_apf_srv) is called. id: " + str(req.id))

        # request
        id = req.id
        xs = req.xs
        ys = req.ys
        xt = req.xt
        yt = req.yt
        name = req.name
        ns = "/r"+str(id)
        heading = np.deg2rad(req.theta)

        # robot object
        robot = Robot(xs, ys, id, name, heading, xt, yt) 

        # update robotic system data
        self.robot_count += 1
        self.model.robot_count += 1
        self.robots_id.append(id)
        self.robots.append(robot)

        # motion_action action **********************************
        print(ns + ": Creating Initial Robot Action: " + ns + "/motion_action ...")
        ac = InitRobotAcion(self.model, robot, self.ax)
        self.ac_services.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.id = id

        return resp
