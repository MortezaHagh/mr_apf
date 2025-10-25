""" create model based on model inputs"""

from typing import List
import numpy as np
from parameters import Params
from model_inputs import ModelInputs


class Map:
    def __init__(self, inputs: ModelInputs):
        self.lim = inputs.lim
        self.x_min = inputs.x_min
        self.y_min = inputs.y_min
        self.x_max = inputs.x_max
        self.y_max = inputs.y_max


class Robot:
    def __init__(self, rid: int, xs: float, ys: float, xt: float, yt: float, r: float = None, heading: float = 0.0, params: Params = None):
        self.rid = rid
        self.ns = "/r"+str(rid)
        self.sns = "r"+str(rid)
        self.xs = xs
        self.ys = ys
        self.xt = xt
        self.yt = yt
        self.heading = np.deg2rad(heading)
        self.priority = rid
        self.r = r
        if self.r is None:
            self.r = params.robot_r  # default robot radius


class ObstacleBase:
    def __init__(self, x: float, y: float, r: float = None, params: Params = None):
        self.x = x
        self.y = y
        self.r = r
        if self.r is None:
            self.r = params.obst_r  # default obstacle radius


class Obstacle(ObstacleBase):
    def __init__(self, x: float, y: float, params: Params, r: float = None):
        super().__init__(x, y, r, params)
        self.d_prec = params.robot_r + self.r + params.d_prec  # 0.57
        self.d_half = 1.5 * self.d_prec
        self.d_start = 2 * self.d_prec
        self.obst_z = 4 * params.fix_f * self.d_prec**4

# class Obstacles:
#     def __init__(self, inputs: ModelInputs):
#         self.r = 0.11
#         self.x = [x for x in inputs.x_obsts]
#         self.y = [y for y in inputs.y_obsts]
#         self.count = len(self.x)


class MRSModel:

    def __init__(self, params: Params):

        print(f"[{self.__class__.__name__}]: Create Base Model")

        # params
        self.params: Params = params

        # model inputs
        inputs = ModelInputs(params)
        self.map_id: int = inputs.map_id

        # Map
        emap = Map(inputs)
        self.emap: Map = emap

        # Obstacles
        self.n_obstacles: int = inputs.n_obsts
        self.obstacles: List[Obstacle] = []
        for i in range(self.n_obstacles):
            x = inputs.x_obsts[i]
            y = inputs.y_obsts[i]
            self.obstacles.append(Obstacle(x, y, params=params))
        # self.obsts: Obstacles = Obstacles(inputs)

        # Robot
        self.n_robots: int = inputs.n_robots
        self.robots: List[Robot] = []
        for i in range(self.n_robots):
            rid = inputs.rids[i]
            h = inputs.headings[i]
            xs = inputs.x_starts[i]
            ys = inputs.y_starts[i]
            xt = inputs.x_targets[i]
            yt = inputs.y_targets[i]
            self.robots.append(Robot(rid, xs, ys, xt, yt, h, params=params))

        # # robots Data
        # self.robots_data: RobotsData = RobotsData(inputs)

# -------------------------------- __main__  -----------------------------------


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from plotter import Plotter
    params = Params()
    model = MRSModel(map_id=1, params=params)
    Plotter(model, params, "")
    plt.show()
