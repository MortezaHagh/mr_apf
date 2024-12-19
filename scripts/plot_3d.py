#! /usr/bin/env python3

import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from create_model import MRSModel



class Plot3D:
    def __init__(self):

        # path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        self.save_path = os.path.join(pkg_path, 'Results-APF/map3d')

        # test name and version
        version = 1
        self.test_id = 7
        self.test_name = "T" + str(self.test_id) + "_v" + str(version)

        # create model
        path_unit = 1  # 0.7
        self.path_unit = 0.7
        n_robots = self.test_id
        self.model = MRSModel(map_id=1,
                                 path_unit=path_unit,
                                 n_robots=n_robots)

        self.init()
        self.plot_f_obstacle()

    def init(self):
        print("plot3D: initializing ... ")
        # settings
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10
        self.obst_r = 0.11 / self.path_unit
        self.prec_d = 0.07 / self.path_unit
        self.robot_r = 0.22 / self.path_unit

        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = 15 * self.obst_prec_d
        self.obst_half_d = 1.5 * self.obst_prec_d
        self.obst_z = 4 * self.fix_f * self.obst_prec_d**4

        self.robot_prec_d = 2 * self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2 * self.robot_prec_d
        self.robot_half_d = 1.5 * self.robot_prec_d
        self.robot_z = 4 * self.fix_f * self.robot_prec_d**4

    def plot_f_obstacle(self):
        print("plot3D: plotting obstacle ... ")

        x_step = 0.1
        y_step = 0.1
        x = np.arange(self.model.map.x_min-2, self.model.map.x_max+5, x_step)
        y = np.arange(self.model.map.y_min-2, self.model.map.y_max+2, y_step)
        X, Y = np.meshgrid(x, y, indexing='ij')
        Z = np.zeros(X.shape)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                xx = X[i, j]
                yy = Y[i, j]

                for k in range(self.model.obst.count):
                    dy = (self.model.obst.y[k] - yy)
                    dx = (self.model.obst.x[k] - xx)
                    d_ro = np.sqrt(dx**2 + dy**2)

                    f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
                    if f > 4:
                        f = 4 + f * 10/1000
                    # f = min(f, 5)
                    f = max(f, 0)
                    Z[i, j] += f

                # Z[i,j] = min(Z[i,j], 20)
                # Z[i,j] = max(Z[i,j], 0)

                dy = (5 - yy)
                dx = (10 - xx)
                d_ro = np.sqrt(dx**2 + dy**2)
                f = self.zeta * 2/d_ro
                # if f>5:
                #      f = 5 + f /10
                f += (self.zeta * (15-d_ro)) * (8/20)
                f = -f
                f = max(f, -10)
                f = min(f, 0)
                Z[i, j] += f

                Z[i, j] = min(Z[i, j], 4)
                Z[i, j] = max(Z[i, j], -10)

        fig = plt.figure()
        # fig, ax = plt.subplots(1,1)
        ax = plt.axes(projection='3d')

        ax.plot_surface(X, Y-2, Z, cmap='viridis', edgecolor='none')
        ax.set_title('Surface plot')

        # ax.contour3D(X, Y, Z, 50, cmap='binary')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')
        # ax.set_zlabel('z')
        ax.set_xlim3d(self.model.map.x_min-7, self.model.map.x_max+9)
        ax.set_ylim3d(self.model.map.y_min-7, self.model.map.y_max+3)
        ax.set_zlim3d(-6, 20)
        # ax.set_xlim((self.model.map.x_min-5, self.model.map.x_max+5))
        # ax.set_ylim((self.model.map.y_min-5, self.model.map.y_max+5))
        # Hide grid lines
        ax.grid(False)
        # Remove the axis
        plt.axis('off')
        # Hide axes ticks
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        # Make panes transparent
        ax.xaxis.pane.fill = False  # Left pane
        ax.yaxis.pane.fill = False  # Right pane

        # plt.savefig(self.save_path+'.png', format='png', dpi=1000)
        # plt.savefig(self.save_path+'.svg', format='svg', dpi=1000)
        plt.show()


if __name__ == '__main__':
    p3 = Plot3D()
