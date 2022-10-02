import numpy as np
import matplotlib.pyplot as plt
from model_inputs import ModelInputs


class Map(object):
    def __init__(self, inputs):
        self.lim = inputs.lim
        self.x_min = inputs.x_min
        self.y_min = inputs.y_min
        self.x_max = inputs.x_max
        self.y_max = inputs.y_max
        self.nx = self.x_max - self.x_min + 1
        self.ny = self.y_max - self.y_min + 1


class Robot(object):
    def __init__(self, map, xs, ys, xt, yt, heading, id, nodes_count):

        self.xs = xs
        self.ys = ys
        self.xt = xt
        self.yt = yt
        self.heading = np.deg2rad(heading)
        self.start_node = (self.ys - map.y_min)*(map.nx) + \
            self.xs+abs(map.x_min)
        self.goal_node = (self.yt - map.y_min)*(map.nx) + \
            self.xt+abs(map.x_min)


class Obstacles(object):
    def __init__(self, map, inputs):
        self.r = 0.25
        self.x = inputs.x_obst
        self.y = inputs.y_obst
        self.count = len(self.x)
        self.nodes = [(y-map.y_min)*map.nx + x + abs(map.x_min)
                      for x, y in zip(self.x, self.y)]


class Nodes(object):
    def __init__(self, map):
        self.count = map.nx*map.ny
        self.x = [x for x in range(map.x_min, map.x_max+1)]*map.ny
        self.y = [y for y in range(map.y_min, map.y_max+1)
                  for x in range(map.nx)]


class CreateModel(object):
    def __init__(self, map_id=4):
        print('Create Base Model')
        inputs = ModelInputs(map_id)

        # Map
        map = Map(inputs)
        self.map = map

        # Obstacles
        self.obst = Obstacles(map, inputs)

        # Nodes
        self.nodes = Nodes(map)

        # Robot
        self.robot_count = inputs.robot_count
        heading = inputs.heading
        xs = inputs.xs
        ys = inputs.ys
        xt = inputs.xt
        yt = inputs.yt
        self.robots = []

        for i in range(self.robot_count):
            self.robots.append(
                Robot(map, xs[i], ys[i], xt[i], yt[i], heading[i], i, self.nodes.count))


if __name__ == '__main__':
    from plot_model import plot_model
    model = CreateModel()
    plot_model(model)
    plt.show()
