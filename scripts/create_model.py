import numpy as np
import matplotlib.pyplot as plt
from model_inputs import ModelInputs


class RobotI(object):
    def __init__(self, inputs):
        self.xs = inputs.xs 
        self.ys = inputs.ys
        self.xt = inputs.xt
        self.yt = inputs.yt
        self.ids = inputs.ids
        self.ns = ["/r"+str(id) for id in inputs.ids]
        self.robot_count = len(inputs.ids)
        
class Map(object):
    def __init__(self, inputs):
        path_unit = 0.5
        self.lim = inputs.lim * path_unit
        self.x_min = inputs.x_min * path_unit
        self.y_min = inputs.y_min * path_unit
        self.x_max = inputs.x_max * path_unit
        self.y_max = inputs.y_max * path_unit

class Robot(object):
    def __init__(self, xs, ys, xt, yt, heading, id):
        path_unit = 0.5
        self.id = id
        self.xs = xs * path_unit
        self.ys = ys * path_unit
        self.xt = xt * path_unit
        self.yt = yt * path_unit
        self.heading = np.deg2rad(heading)

class Obstacles(object):
    def __init__(self, map, inputs):
        path_unit = 0.5
        self.r = 0.25
        self.x = [x*path_unit  for x in inputs.x_obst]
        self.y = [y*path_unit  for y in inputs.y_obst]
        self.count = len(self.x)

# ---------------------------- CreateModel ----------------------------------
class CreateModel(object):
    def __init__(self, map_id=-1):

        print('Create Base Model')
        
        # model inputs
        inputs = ModelInputs(map_id)

        #
        self.path_unit = 0.5

        # Map
        map = Map(inputs)
        self.map = map

        # Obstacles
        self.obst = Obstacles(map, inputs)

        # Robot
        self.robot_count = inputs.robot_count
        heading = inputs.heading
        xs = inputs.xs
        ys = inputs.ys
        xt = inputs.xt
        yt = inputs.yt
        self.robots = []

        for i in range(self.robot_count):
            self.robots.append(Robot(xs[i], ys[i], xt[i], yt[i], heading[i], i))

        # robot I
        self.robots_i = RobotI(inputs)

# -------------------------------- __main__  -----------------------------------

if __name__ == '__main__':
    from plot_model import plot_model
    model = CreateModel()
    plot_model(model)
    plt.show()
