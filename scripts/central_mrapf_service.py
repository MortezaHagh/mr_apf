#! /usr/bin/env python

""" central service,  includes fleet_data_handler, and create a RobotPlanner for each request """
from typing import List
import numpy as np
import rospy
from parameters import Params
from create_model import MRSModel, Robot
from fleet_data_handler import FleetDataHandler
from robot_planner_l0_ac import RobotPlannerAc
from apf.srv import InitRobot, InitRobotResponse, InitRobotRequest


class CentralMRAPF:

    def __init__(self, model: MRSModel, central_mrapf_srv_name: str):
        rospy.loginfo(f"[{self.__class__.__name__}]: Initializing Central MRAPF Path Planning Service Server.")

        # data
        self.model: MRSModel = model

        # all robots
        self.n_robots: int = 0
        self.robots: List[Robot] = []
        self.planners: List[RobotPlannerAc] = []
        self.robot_ids: List[int] = []

        # send_robot_update_srv
        self.fleet_data_handler = FleetDataHandler(model.params)

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
        self.fleet_data_handler.add_robot(rid, robot.sns)
        self.fleet_data_handler.update_goal(rid, robot.xt, robot.yt)

        # motion_action action *************************************************
        rospy.loginfo(f"[{self.__class__.__name__}]: Creating Initial Robot Action: {robot.ns}/motion_action ...")
        ac = RobotPlannerAc(self.model, robot)
        self.planners.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.rid = rid
        return resp
