""" create model based on model inputs"""

from typing import List
import numpy as np
from parameters import Params
from model_inputs import ModelInputs


# class RobotsData:

#     def __init__(self, inputs: ModelInputs, path_unit: float):
#         self.x_starts = [x*path_unit for x in inputs.x_starts]
#         self.ys = [y*path_unit for y in inputs.y_starts]
#         self.xt = [x*path_unit for x in inputs.x_targets]
#         self.yt = [y*path_unit for y in inputs.y_targets]
#         self.heading = list(inputs.headings)
#         self.rids: List[int] = inputs.rids
#         self.ns = ["/r"+str(id) for id in inputs.rids]
#         self.n_robots = len(inputs.rids)


class Map:
    def __init__(self, inputs: ModelInputs, path_unit: float):
        self.lim = inputs.lim * path_unit
        self.x_min = inputs.x_min * path_unit
        self.y_min = inputs.y_min * path_unit
        self.x_max = inputs.x_max * path_unit
        self.y_max = inputs.y_max * path_unit


class Robot:
    def __init__(self, rid: int, xs: float, ys: float, xt: float, yt: float, r: float = None, heading: float = 0.0, path_unit: float = 1.0):
        self.rid = rid
        self.ns = "/r"+str(rid)
        self.sns = "r"+str(rid)
        self.xs = xs * path_unit
        self.ys = ys * path_unit
        self.xt = xt * path_unit
        self.yt = yt * path_unit
        self.heading = np.deg2rad(heading)
        self.priority = rid
        self.r = r
        if self.r is None:
            self.r = 0.22  # default robot radius


class Obstacle:
    def __init__(self, x: float, y: float, r: float = None, path_unit: float = 1.0):
        self.x = x * path_unit
        self.y = y * path_unit
        self.r = r
        if self.r is None:
            self.r = 0.11  # default obstacle radius


# class Obstacles:
#     def __init__(self, inputs: ModelInputs, path_unit: float):
#         self.r = 0.11
#         self.x = [x*path_unit for x in inputs.x_obsts]
#         self.y = [y*path_unit for y in inputs.y_obsts]
#         self.count = len(self.x)


class MRSModel:

    def __init__(self, path_unit: float = 1.0, params: Params = None):

        print(f"[{self.__class__.__name__}]: Create Base Model")

        # params
        self.params: Params = params

        # model inputs
        inputs = ModelInputs(params.map_id, path_unit, params.nr)
        self.map_id: int = inputs.map_id

        # self.path_unit = path_unit
        path_unit = 1.0

        # Map
        emap = Map(inputs, path_unit)
        self.emap: Map = emap

        # Obstacles
        self.n_obstacles: int = inputs.n_obsts
        self.obstacles: List[Obstacle] = []
        for i in range(self.n_obstacles):
            x = inputs.x_obsts[i]
            y = inputs.y_obsts[i]
            self.obstacles.append(Obstacle(x, y))
        # self.obsts: Obstacles = Obstacles(inputs, path_unit)

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
            self.robots.append(Robot(rid, xs, ys, xt, yt, h, path_unit))

        # # robots Data
        # self.robots_data: RobotsData = RobotsData(inputs, path_unit)

# -------------------------------- __main__  -----------------------------------


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from plotter import Plotter
    params = Params()
    model = MRSModel(map_id=1, params=params)
    Plotter(model, params, "")
    plt.show()
