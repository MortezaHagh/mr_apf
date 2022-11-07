#! /usr/bin/env python

# pyright: reportMissingImports=false

import rospy
import numpy as np
from parameters import Params
from pose_service import PoseService 
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
        self.priority = id

    # -------------------------------- InitRobotService -------------------------- #

class InitRobotService(object):
    def __init__(self, model, init_srv_name):

        # data
        self.model = model

        # all robots
        self.robots = []
        self.robots_id = []
        self.robot_count = 0
        self.ac_services = []

        # pose service
        posr_srv_name = "/pose_service"
        self.pose_srv = PoseService(posr_srv_name)

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
        # self.model.robot_count += 1
        self.robots_id.append(id)
        self.robots.append(robot)

        # setting - parameters
        self.name_s = '/r' + str(id)
        action_params = Params(id)
        action_params.set_name_space(self.name_s)

        # update pose service
        self.pose_srv.count += 1
        self.pose_srv.xt[id] = 0
        self.pose_srv.yt[id] = 0
        self.pose_srv.ids.append(id)
        self.pose_srv.priorities[id] = robot.priority
        self.pose_srv.topics[id] = action_params.lis_topic

        # motion_action action **********************************
        print(ns + ": Creating Initial Robot Action: " + ns + "/motion_action ...")
        ac = InitRobotAcion(self.model, robot, action_params)
        self.ac_services.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.id = id

        return resp
