#! /usr/bin/env python3

import rospy
import numpy as np
from matplotlib.pylab import plt
from model_inputs import ModelInputs
from create_model import CreateModel
from plot_model import plot_model

class APF(object):
    def __init__(self):

        # model
        self.model = CreateModel(map_id=4)


        #
        self.obs_r = 1
        self.dt = 0.05
        self.zeta = 1
        self.d_rt = 1000

        #
        self.v = 0.5
        self.v_max = 0.5
        self.v_min = 0
        self.w_max = 0.3
        self.w_min = -0.3

        #
        self.xx = []
        self.yy = []

        # map
        self.map()

        # move
        self.move()

        # plot
        self.plotting()

    # --------------------------------------------------------#

    def move(self):
        self.get_robot()
        while self.d_rt > 1:
            [f_r, f_theta, phi] = self.forces()

            vt = self.v*self.dt
            theta = self.r_theta + phi
            self.r_x += vt * np.cos(theta)
            self.r_y += vt * np.sin(theta)
            self.r_theta = self.modify_angle(theta)
            self.xx.append(self.r_x)
            self.yy.append(self.r_y)

    # --------------------------------------------------------------#

    def forces(self):
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]

        phi = np.arctan2(f_theta, f_r)
        phi = round(phi, 2)
        print([f_r, f_theta, phi])
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

    # --------------------------------------------------------------#

    def get_robot(self):
        self.r_x = 0
        self.r_y = 0
        self.r_theta = 0

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2*np.pi) - np.pi
        return theta_mod

    def modify_vel(self, v, w):
        v = min(v, self.v_max)
        v = max(v, self.v_min)
        w = min(w, self.w_max)
        w = max(w, self.w_min)
        return [v, w]

    def map(self):
        self.lim = 10

        # robot target
        self.t_x = 10
        self.t_y = 10

        # obstacles
        self.obs_n = len(self.model.obst.x)
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y

    # --------------------------------------------------------------#

    def plotting(self):
        fig, ax = plot_model(self.model)
        
        #
        ax.plot(self.xx, self.yy)
        plt.show()


if __name__ == "__main__":
    # rospy.init_node("main_node")
    apf = APF()
    apf.f_obstacle()
    apf.f_target()
    # print(apf.obs_f)
    # print(apf.target_f)
    # print(".........")
