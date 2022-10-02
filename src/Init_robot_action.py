#! /usr/bin/env python3

import rospy
import actionlib
import numpy as np
from apf.msg import InitRobotAction, InitRobotResult

class InitRobotAcion(object):
    def __init__(self, model, i, name, settings, velocities):
        
        self.path_x = []
        self.path_y = []
        
        self.i = i
        self.model = model
        self.action_name = name

        self.dt = settings["dt"]
        self.zeta = settings["zeta"]
        self.d_rt = settings["d_rt"]
        self.obs_r = settings["obs_r"]

        self.v = velocities["v"]

        self.map()

        self.ac_ = actionlib.SimpleActionServer(self.action_name, InitRobotAction, self.exec_cb)
        self.ac_.start()

    def exec_cb(self, goal):

        # move
        self.move()

        # result
        res = InitRobotResult()
        res.result = True
        res.path_x = self.path_x
        res.path_y = self.path_y
        self.ac_.set_succeeded(res)

    # --------------------------  move  ---------------------------#

    def move(self):
        self.get_robot()
        while self.d_rt > 0.25:
            [f_r, f_theta, phi] = self.forces()

            vt = self.v*self.dt
            theta = self.r_theta + phi
            self.r_x += vt * np.cos(theta)
            self.r_y += vt * np.sin(theta)
            self.r_theta = self.modify_angle(theta)
            self.path_x.append(self.r_x)
            self.path_y.append(self.r_y)

    # -----------------------  forces  ----------------------------#

    def forces(self):
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]

        phi = np.arctan2(f_theta, f_r)
        phi = round(phi, 2)
        return [f_r, f_theta, phi]

    def f_target(self):
        dx = self.t_x - self.r_x
        dy = self.t_y - self.r_y
        d_rt = np.sqrt(dx**2+dy**2)
        f = self.zeta * d_rt
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.d_rt = d_rt
        fx = round(f*np.cos(angle_diff), 2)
        fy = round(f*np.sin(angle_diff), 2)
        self.target_f = [fx, fy]

    def f_obstacle(self):
        self.obs_f = [0, 0]
        for i in range(self.obs_n):
            dy = self.obs_y[i]-self.r_y
            dx = self.obs_x[i]-self.r_x
            dy = -dy
            dx = -dx
            d_ro = np.sqrt(dx**2+dy**2)
            if d_ro >= self.obs_r:
                f = 0
                theta = 0
            else:
                f = ((self.zeta/0.01)*((1/d_ro)-(1/self.obs_r))**2)*(1/d_ro)**2
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                self.obs_f[0] += round(templ[0], 2)
                self.obs_f[1] += round(templ[1], 2)

    # -------------------------  get_robot, modify_angle  ------------------------------#

    def get_robot(self):
        self.r_x = self.model.robots[self.i].xs
        self.r_y = self.model.robots[self.i].ys
        self.r_theta = self.model.robots[self.i].heading

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2*np.pi) - np.pi
        return theta_mod
    
    def map(self):

        # robot target
        self.t_x = self.model.robots[self.i].xt
        self.t_y = self.model.robots[self.i].yt

        # obstacles
        self.obs_n = self.model.obst.count
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y
