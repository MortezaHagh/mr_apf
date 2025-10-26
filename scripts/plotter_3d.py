#! /usr/bin/env python3

import os
import rospkg  # type: ignore
import numpy as np
import matplotlib.pyplot as plt  # type: ignore
import plotly.graph_objects as go  # type: ignore
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
        self.p.map_id = 3
        self.model: MRSModel = MRSModel(params=self.p)

        print("plot3D: initializing ... ")
        self.plot_forces()

    def _compute_obstacles_field(self, X, Y, Z):

        for obst in self.model.obstacles:
            # compute distance grid (vectorized)
            dx = X - obst.x
            dy = Y - obst.y
            d_ro = np.hypot(dx, dy)
            # avoid division by zero
            d_ro = np.maximum(d_ro, 1e-8)

            # vectorized formula, then clamp

            # f1 : gaussian
            sigma = obst.d_prec / 3
            h = obst.d_prec * 4
            f1 = h * (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((d_ro - 0) / sigma) ** 2)
            Z += f1

            # f = (obst.obst_z * 1) * ((1.0 / d_ro) - (1.0 / obst.d_start)) ** 2 * (1.0 / d_ro) ** 2
            # f = np.clip(f, 0, 5)
            # Z += f

        return Z

    def _compute_goal_field(self, X, Y, Z):
        # compute distance grid (vectorized)
        xg = self.model.emap.x_max / 2.0
        yg = self.model.emap.y_max / 2.0
        dx = X - xg
        dy = Y - yg
        d_rg = np.hypot(dx, dy)
        # avoid division by zero
        d_rg = np.maximum(d_rg, 1e-8)

        # f : gaussian
        sigma = 2.0
        h = sigma * 5.0
        f1 = -h * (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((d_rg - 0) / sigma) ** 2)
        Z += f1
        return Z

    def plot_plotly(self, X, Y, Z):
        surface = go.Surface(
            x=X,
            y=Y,  # keep the same Y-offset as original
            z=Z,
            colorscale='Viridis',
            showscale=True,
            cmin=0,
            cmax=20,
            lighting=dict(ambient=0.6, diffuse=0.6, roughness=0.9, specular=0.1),
            hoverinfo='skip'
        )

        fig = go.Figure(data=[surface])
        fig.update_layout(
            title='Potential Field (interactive)',
            margin=dict(l=0, r=0, t=40, b=0),
            scene=dict(
                xaxis=dict(showgrid=False, showticklabels=False, visible=False,
                           range=[self.model.emap.x_min - 7, self.model.emap.x_max + 9]),
                yaxis=dict(showgrid=False, showticklabels=False, visible=False,
                           range=[self.model.emap.y_min - 7, self.model.emap.y_max + 3]),
                zaxis=dict(showgrid=False, showticklabels=False, visible=False, range=[-6, 20]),
                aspectmode='manual',
                aspectratio=dict(x=1.2, y=1.0, z=0.5),
            )
        )

        # initial camera for a nicer 3D view
        fig.update_layout(scene_camera=dict(eye=dict(x=1.5, y=1.5, z=0.8)))
        fig.show()

    def plot_matplotlib(self, X, Y, Z):
        fig = plt.figure(figsize=(10, 7))
        ax = plt.axes(projection='3d')
        # rcount/ccount reduce data for faster rendering if grid is very dense
        ax.plot_surface(X, Y, Z, alpha=.9, linewidth=0, cmap='viridis', edgecolor='none')
        # ax.plot_surface(X, Y, Z, alpha=.9, linewidth=0, cmap='viridis', edgecolor='none', rcount=200, ccount=200)
        # ax.plot_surface(X, Y, Z, alpha=.9, linewidth=0, cmap=plt.cm.gist_earth, antialiased=False,
        #                 shade=False, rstride=1, cstride=1)
        ax.set_title('Surface plot')
        ax.set_xlim3d(self.model.emap.x_min - 7, self.model.emap.x_max + 9)
        ax.set_ylim3d(self.model.emap.y_min - 7, self.model.emap.y_max + 3)
        ax.set_zlim3d(-10, 10)
        ax.grid(False)
        plt.axis('off')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        # Make panes transparent where supported
        try:
            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
        except Exception:
            pass
        plt.show()

    def plot_forces(self, step: float = 0.1):
        print("plot3D: plotting forces ... ")

        # build grid (same extents as original)
        x = np.arange(self.model.emap.x_min - 1, self.model.emap.x_max + 1, step)
        y = np.arange(self.model.emap.y_min - 1, self.model.emap.y_max + 1, step)
        X, Y = np.meshgrid(x, y, indexing='ij')  # keep indexing consistent with original
        Z = np.zeros_like(X, dtype=float)
        Z = self._compute_obstacles_field(X, Y, Z)
        Z = self._compute_goal_field(X, Y, Z)

        #
        self.plot_matplotlib(X, Y, Z)
        # self.plot_plotly(X, Y, Z)
        # try:
        #     self.plot_plotly(X, Y, Z)
        # except Exception as e:
        #     print("plot3D: failed to create interactive plot, falling back to Matplotlib.")
        #     self.plot_matplotlib(X, Y, Z)


if __name__ == '__main__':
    Plot3D()
