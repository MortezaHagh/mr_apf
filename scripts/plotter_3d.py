#! /usr/bin/env python3

import os
import rospkg  # type: ignore
import numpy as np
import matplotlib.pyplot as plt
from create_model import MRSModel
from parameters import Params


class Plot3D:
    def __init__(self):

        # path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        self.save_path = os.path.join(pkg_path, 'results/emap3d')

        # create model
        self.p = Params()
        self.model: MRSModel = MRSModel(params=self.p)

        self.init()
        self.plot_f_obstacle()

    def init(self):
        print("plot3D: initializing ... ")

    def plot_f_obstacle(self):
        print("plot3D: plotting obstacle ... ")

        step = 0.05
        x_step = step
        y_step = step
        x = np.arange(self.model.emap.x_min-1, self.model.emap.x_max+1, x_step)
        y = np.arange(self.model.emap.y_min-1, self.model.emap.y_max+1, y_step)
        X, Y = np.meshgrid(x, y, indexing='ij')
        Z = np.zeros(X.shape)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                xx = X[i, j]
                yy = Y[i, j]

                for k, obst in enumerate(self.model.obstacles):
                    dx = obst.x - xx
                    dy = obst.y - yy
                    d_ro = np.sqrt(dx**2 + dy**2)

                    f = ((obst.obst_z * 1) * ((1 / d_ro) - (1 / obst.d_start))**2) * (1 / d_ro)**2
                    f = min(f, 10)
                    f = max(f, 0)
                    Z[i, j] += f

                # Z[i,j] = min(Z[i,j], 20)
                # Z[i,j] = max(Z[i,j], 0)

        fig = plt.figure()
        # fig, ax = plt.subplots(1,1)
        ax = plt.axes(projection='3d')

        ax.plot_surface(X, Y-2, Z, cmap='viridis', edgecolor='none')
        ax.set_title('Surface plot')

        # ax.contour3D(X, Y, Z, 50, cmap='binary')
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')
        # ax.set_zlabel('z')
        ax.set_xlim3d(self.model.emap.x_min-7, self.model.emap.x_max+9)
        ax.set_ylim3d(self.model.emap.y_min-7, self.model.emap.y_max+3)
        ax.set_zlim3d(-6, 20)
        # ax.set_xlim((self.model.emap.x_min-5, self.model.emap.x_max+5))
        # ax.set_ylim((self.model.emap.y_min-5, self.model.emap.y_max+5))
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
