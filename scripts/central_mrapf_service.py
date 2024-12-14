#! /usr/bin/env python

""" central service,  create RobotPlanner for each request """

import rospy
import numpy as np
from typing import List
from parameters import Params
from pose_service import PoseService
from create_model import MRSModel
from robot_planner_server import RobotPlanner
from apf.srv import InitRobot, InitRobotResponse, InitRobotRequest


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


class CentralMRAPF(object):
    n_robots: int
    model: MRSModel
    robots: List[Robot]
    robot_ids: List[int]
    planners: List[RobotPlanner]
    pose_srv: PoseService

    def __init__(self, model, central_mrapf_srv_name):

        # data
        self.model = model

        # all robots
        self.n_robots = 0
        self.robots = []
        self.robot_ids = []
        self.planners = []

        # pose service
        posr_srv_name = "/pose_service"
        self.pose_srv = PoseService(posr_srv_name)

        # init_robot_srv Service Server
        self.robot_srv = rospy.Service(central_mrapf_srv_name, InitRobot, self.apf_srv_callback)
        print("Central Service Server (central_mrapf_srv) is created. ")

    def apf_srv_callback(self, req: InitRobotRequest):

        print("[central_mrapf_srv] is called. req id: " + str(req.id))

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
        self.n_robots += 1
        self.robot_ids.append(id)
        self.robots.append(robot)

        # setting - parameters
        self.name_s = '/r' + str(id)
        action_params = Params(id)
        action_params.set_name_space(self.name_s)

        # update pose service
        self.pose_srv.add_robot(id, robot.priority, action_params.lis_topic)
        self.pose_srv.topics[id] = action_params.lis_topic

        # motion_action action **********************************
        print(ns + ": Creating Initial Robot Action: " + ns + "/motion_action ...")
        ac = RobotPlanner(self.model, robot, action_params)
        self.planners.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.id = id
        return resp
