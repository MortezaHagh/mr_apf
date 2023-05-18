import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from model_inputs import ModelInputs


class RobotI(object):
    def __init__(self, inputs, path_unit):
        self.xs = [x*path_unit for x in inputs.xs] 
        self.ys = [y*path_unit for y in inputs.ys]
        self.xt = [x*path_unit for x in inputs.xt]
        self.yt = [y*path_unit for y in inputs.yt]
        self.heading = [h for h in inputs.heading]
        self.ids = inputs.ids
        self.ns = ["/r"+str(id) for id in inputs.ids]
        self.robot_count = len(inputs.ids)
        
class Map(object):
    def __init__(self, inputs, path_unit):
        self.lim = inputs.lim * path_unit
        self.x_min = inputs.x_min * path_unit
        self.y_min = inputs.y_min * path_unit
        self.x_max = inputs.x_max * path_unit
        self.y_max = inputs.y_max * path_unit

class Robot(object):
    def __init__(self, xs, ys, xt, yt, heading, id, path_unit):
        self.id = id
        self.xs = xs * path_unit
        self.ys = ys * path_unit
        self.xt = xt * path_unit
        self.yt = yt * path_unit
        self.heading = np.deg2rad(heading)

class Obstacles(object):
    def __init__(self, map, inputs, path_unit):
        self.r = 0.25
        self.x = [x*path_unit  for x in inputs.x_obst]
        self.y = [y*path_unit  for y in inputs.y_obst]
        self.count = len(self.x)

# ---------------------------- CreateModel ----------------------------------
class CreateModel(object):
    def __init__(self, map_id=1, path_unit=1.0, robot_count = 1):

        print('Create Base Model')
        
        # model inputs
        inputs = ModelInputs(map_id, path_unit, robot_count)
        self.map_ind = inputs.map_ind

        #
        self.path_unit = path_unit
        path_unit = 1.0

        # Map
        map = Map(inputs, path_unit)
        self.map = map

        # Obstacles
        self.obst_count_orig = inputs.obst_count_orig
        self.obst = Obstacles(map, inputs, path_unit)

        # Robot
        self.robot_count = inputs.robot_count
        heading = inputs.heading
        xs = inputs.xs
        ys = inputs.ys
        xt = inputs.xt
        yt = inputs.yt
        self.robots = []

        for i in range(self.robot_count):
            self.robots.append(Robot(xs[i], ys[i], xt[i], yt[i], heading[i], i, path_unit))

        # robot I
        self.robots_i = RobotI(inputs, path_unit)

# -------------------------------- __main__  -----------------------------------

if __name__ == '__main__':
    from plot_model import plot_model
    
    class Setting:
        def __init__(self):
            self.obst_r = 0.11
            self.obst_prec_d = 0.4
    setting = Setting()
    
    for i in range(1, 13):
        model = CreateModel(robot_count=i)
        plot_model(model, setting)

        # save fig
        ind = str(model.map_ind)
        o_ind = str(model.obst_count_orig)
        no = 'o'+o_ind+'_map'+ind
        map_name = 'maps/'+no #+ '_e'
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)
        plt.savefig(filename+'.svg', format="svg", dpi=1000)
        plt.savefig(filename+'.png', format="png", dpi=500)

    # plt.show()
