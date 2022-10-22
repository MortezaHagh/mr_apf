#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from apf.srv import MyPose, MyPoseRequest
from apf.msg import InitRobotAction, InitRobotResult, InitRobotFeedback


class InitRobotAcion(object):
    def __init__(self, model, ind, name, settings, velocities):

        # ros
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.shutdown_hook)

        #
        self.feedback = InitRobotFeedback()
        self.res = InitRobotResult()
        self.path_x = []
        self.path_y = []

        self.ind = ind
        self.model = model
        self.action_name = name

        self.dt = settings["dt"]
        self.zeta = settings["zeta"]
        self.robot_r = settings["robot_r"]
        self.obs_effect_r = settings["obs_effect_r"]
        self.pose_srv_name = settings["pose_srv_name"]
        self.goal_distance = settings["goal_distance"]

        self.v = velocities["v"]

        # map: target and obstacles coordinates
        self.map()

        # get robots start coords 
        self.get_robot()

        # pose service client
        rospy.wait_for_service(self.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(self.pose_srv_name, MyPose)

        self.ac_ = actionlib.SimpleActionServer(self.action_name, InitRobotAction, self.exec_cb)
        self.ac_.start()

    # --------------------------  exec_cb  ---------------------------#

    def exec_cb(self, goal):

        # move
        self.move()

        # result
        self.res.result = True
        self.res.path_x = self.path_x
        self.res.path_y = self.path_y
        self.ac_.set_succeeded(self.res)
        return

    # --------------------------  move  ---------------------------#

    def move(self):
        self.get_robot()
        while self.goal_distance > 0.25 and not rospy.is_shutdown():
            [f_r, f_theta, phi] = self.forces()

            vt = self.v*self.dt
            theta = self.r_theta + phi
            self.r_x += vt * np.cos(theta)
            self.r_y += vt * np.sin(theta)
            self.r_theta = self.modify_angle(theta)

            # result
            self.path_x.append(self.r_x)
            self.path_y.append(self.r_y)

            # # feedback
            # self.feedback.path = [self.r_x, self.r_y]
            # self.ac_.publish_feedback(self.feedback)

            self.rate.sleep()

    # -----------------------  forces  ----------------------------#

    def forces(self):
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]

        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]

        phi = np.arctan2(f_theta, f_r)
        phi = round(phi, 2)
        return [f_r, f_theta, phi]

    def f_robots(self):
        req = MyPoseRequest()
        req.ind = self.ind
        resp = self.pose_client(req.ind)
        self.robot_f = [0, 0]
        for i in range(resp.count):
            dx = resp.x[i]-self.r_x
            dy = resp.y[i]-self.r_y
            dy = -dy
            dx = -dx
            d_ro = np.sqrt(dx**2+dy**2)
            if d_ro >= self.robot_r:
                f = 0
                theta = 0
            else:
                f = ((self.zeta/0.01)*((1/d_ro)-(1/self.obs_effect_r))**2)*(1/d_ro)**2
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                self.robot_f[0] += round(templ[0], 2)
                self.robot_f[1] += round(templ[1], 2)

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_distance = np.sqrt(dx**2+dy**2)
        f = self.zeta * goal_distance
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_distance = goal_distance
        fx = round(f*np.cos(angle_diff), 2)
        fy = round(f*np.sin(angle_diff), 2)
        self.target_f = [fx, fy]

    def f_obstacle(self):
        self.obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i]-self.r_y)
            dx = -(self.obs_x[i]-self.r_x)
            d_ro = np.sqrt(dx**2+dy**2)
            if d_ro >= self.obs_effect_r:
                f = 0
                theta = 0
            else:
                f = ((self.zeta*100)*((1/d_ro)-(1/self.obs_effect_r))**2)*(1/d_ro)**2  # mh 100
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                self.obs_f[0] += round(templ[0], 2)
                self.obs_f[1] += round(templ[1], 2)

    # -------------------------  get_robot, modify_angle  ------------------------------#

    def get_robot(self):
        self.r_x = self.model.robots[self.ind].xs
        self.r_y = self.model.robots[self.ind].ys
        self.r_theta = self.model.robots[self.ind].heading

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2*np.pi) - np.pi
        return theta_mod

    def map(self):

        # robot target
        self.goal_x = self.model.robots[self.ind].xt
        self.goal_y = self.model.robots[self.ind].yt

        # obstacles
        self.obs_count = self.model.obst.count
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y

    def shutdown_hook(self):
        print("shutting down from robot action")
