#! /usr/bin/env python
""" central service,  create RobotPlanner for each request """

from typing import List
import numpy as np
import rospy
from parameters import Params
from create_model import MRSModel, Robot
from fleet_data import FleetDataH
from robot_planner_l0_ac import RobotPlannerAc
from apf.srv import InitRobot, InitRobotResponse, InitRobotRequest


class CentralMRAPF:
    n_robots: int
    robot_ids: List[int]
    model: MRSModel
    fleet_data_h: FleetDataH
    robots: List[Robot]
    planners: List[RobotPlannerAc]

    def __init__(self, model: MRSModel, central_mrapf_srv_name: str):
        rospy.loginfo(f"[{self.__class__.__name__}]: Initializing Central MRAPF Path Planning Service Server.")

        # data
        self.model = model

        # all robots
        self.n_robots = 0
        self.robots = []
        self.planners = []
        self.robot_ids = []

        # send_robot_update_srv
        params = Params(-1)
        self.fleet_data_h = FleetDataH(params)

        # init_robot_srv Service Server
        self.robot_srv = rospy.Service(central_mrapf_srv_name, InitRobot, self.apf_srv_callback)
        rospy.loginfo(f"[{self.__class__.__name__}]: Central Service Server (central_mrapf_srv) is created. ")

    def apf_srv_callback(self, req: InitRobotRequest) -> InitRobotResponse:

        rospy.loginfo(f"[{self.__class__.__name__}]: 'central_mrapf_srv' is called. req rid: " + str(req.rid))

        # robot object
        rid = req.rid
        heading = np.deg2rad(req.theta)
        robot = Robot(req.rid, req.xs, req.ys, heading, req.xt, req.yt)

        # update robotic system data
        self.n_robots += 1
        self.robot_ids.append(rid)
        self.robots.append(robot)

        # update fleet data handler
        self.fleet_data_h.add_robot(rid, robot.sns)
        self.fleet_data_h.update_goal(rid, robot.xt, robot.yt)

        # motion_action action *************************************************
        rospy.loginfo(f"[{self.__class__.__name__}]: Creating Initial Robot Action: {robot.ns}/motion_action ...")
        ac = RobotPlannerAc(self.model, robot)
        self.planners.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.rid = rid
        return resp
