""" create model based on model inputs"""

from typing import List
import numpy as np
from parameters import Params
from model_inputs import ModelInputs


class RobotsData:

    def __init__(self, inputs: ModelInputs, path_unit: float):
        self.xs = [x*path_unit for x in inputs.xs]
        self.ys = [y*path_unit for y in inputs.ys]
        self.xt = [x*path_unit for x in inputs.xt]
        self.yt = [y*path_unit for y in inputs.yt]
        self.heading = list(inputs.heading)
        self.rids: List[int] = inputs.rids
        self.ns = ["/r"+str(id) for id in inputs.rids]
        self.n_robots = len(inputs.rids)


class Map:
    def __init__(self, inputs: ModelInputs, path_unit: float):
        self.lim = inputs.lim * path_unit
        self.x_min = inputs.x_min * path_unit
        self.y_min = inputs.y_min * path_unit
        self.x_max = inputs.x_max * path_unit
        self.y_max = inputs.y_max * path_unit


class Robot:
    def __init__(self, rid: int = -1, xs: float = None, ys: float = None, xt: float = None, yt: float = None, heading: float = 0.0, path_unit: float = 1.0):
        self.rid = rid
        self.ns = "/r"+str(rid)
        self.sns = "r"+str(rid)
        self.xs = xs * path_unit
        self.ys = ys * path_unit
        self.xt = xt * path_unit
        self.yt = yt * path_unit
        self.heading = np.deg2rad(heading)
        self.priority = rid


class Obstacles:
    def __init__(self, inputs: ModelInputs, path_unit: float):
        self.r = 0.25
        self.x = [x*path_unit for x in inputs.x_obst]
        self.y = [y*path_unit for y in inputs.y_obst]
        self.count = len(self.x)


class MRSModel:

    def __init__(self, map_id: int = 1, path_unit: float = 1.0, n_robots: int = 1):

        print(f"[{self.__class__.__name__}]: Create Base Model")

        # model inputs
        inputs = ModelInputs(map_id, path_unit, n_robots)
        self.map_id: int = inputs.map_id

        #
        # self.path_unit = path_unit
        path_unit = 1.0

        # Map
        emap = Map(inputs, path_unit)
        self.emap: Map = emap

        # Obstacles
        self.n_obst_orig: int = inputs.n_obst_orig
        self.obsts: Obstacles = Obstacles(inputs, path_unit)

        # Robot
        self.n_robots: int = inputs.n_robots
        heading = inputs.heading
        xs = inputs.xs
        ys = inputs.ys
        xt = inputs.xt
        yt = inputs.yt
        self.robots: List[Robot] = []

        #
        for i in range(self.n_robots):
            self.robots.append(Robot(i, xs[i], ys[i], xt[i], yt[i], heading[i], path_unit))

        # robots Data
        self.robots_data: RobotsData = RobotsData(inputs, path_unit)

# -------------------------------- __main__  -----------------------------------


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from plotter import Plotter
    params = Params()
    model = MRSModel(map_id=1)
    Plotter(model, params, "")
    plt.show()
