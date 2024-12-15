#! /usr/bin/env python
""" central service,  create RobotPlanner for each request """

from typing import List
import numpy as np
import rospy
from parameters import Params
from create_model import MRSModel
from pose_service import PoseService
from mrapf_classes import PlannerRobot
from robot_planner_server import RobotPlanner
from apf.srv import InitRobot, InitRobotResponse, InitRobotRequest


class CentralMRAPF:
    n_robots: int
    robot_ids: List[int]
    model: MRSModel
    pose_srv: PoseService
    robots: List[PlannerRobot]
    planners: List[RobotPlanner]

    def __init__(self, model: MRSModel, central_mrapf_srv_name: str):
        rospy.loginfo(f"[{self.__class__.__name__}]: Initializing Central MRAPF Path Planning Service Server.")

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
        rospy.loginfo(f"[{self.__class__.__name__}]: Central Service Server (central_mrapf_srv) is created. ")

    def apf_srv_callback(self, req: InitRobotRequest):

        rospy.loginfo(f"[{self.__class__.__name__}]: 'central_mrapf_srv' is called. req id: " + str(req.id))

        # robot object
        rid = req.id
        heading = np.deg2rad(req.theta)
        robot = PlannerRobot(req.xs, req.ys, req.id, req.name, heading, req.xt, req.yt)

        # update robotic system data
        self.n_robots += 1
        self.robot_ids.append(rid)
        self.robots.append(robot)

        # setting - parameters
        name_s = "/r" + str(rid)
        action_params = Params(rid)
        action_params.set_name_space(name_s)

        # update pose service
        self.pose_srv.add_robot(rid, robot.priority, action_params.lis_topic)
        self.pose_srv.topics[rid] = action_params.lis_topic

        # motion_action action *************************************************
        rospy.loginfo(f"[{self.__class__.__name__}]: Creating Initial Robot Action: {name_s}/motion_action ...")
        ac = RobotPlanner(self.model, robot, action_params)
        self.planners.append(ac)

        # service responce
        resp = InitRobotResponse()
        resp.success = True
        resp.id = rid
        return resp
