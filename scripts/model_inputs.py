""" model inputs - obstacles - robots """
from typing import List
import os
import json
import random
import numpy as np
import rospkg  # type: ignore
from parameters import Params
from my_utils import cal_distance


class ModelInputs:

    def __init__(self, params: Params):
        print(f"[{self.__class__.__name__}] Inputs for creating model")
        self.n_robots: int = params.nr
        self.map_id: int = params.map_id
        self.path_unit: float = params.path_unit
        #
        self.lim: int = None
        self.x_min: int = None
        self.y_min: int = None
        self.x_max: int = None
        self.y_max: int = None
        self.rids: List[int] = None
        self.x_starts: List[float] = None
        self.y_starts: List[float] = None
        self.x_targets: List[float] = None
        self.y_targets: List[float] = None
        self.n_obsts: int = None
        self.x_obsts: List[float] = None
        self.y_obsts: List[float] = None
        self.headings: List[float] = None
        #
        if self.map_id == 1:
            self.map_0(self.n_robots)
            # self.collide()
            # self.obstacles2()
            # self.random_map_2(n_robots)
            # self.from_json_file(n_robots, path_unit)
        #
        self.rids = list(range(0, self.n_robots))
        self.apply_path_unit(self.path_unit)

    def apply_path_unit(self, path_unit: float = 1.0):
        # map
        self.lim = self.lim * path_unit
        self.x_min = self.x_min * path_unit
        self.y_min = self.y_min * path_unit
        self.x_max = self.x_max * path_unit
        self.y_max = self.y_max * path_unit
        # obstcles
        self.x_obsts = [x * path_unit for x in self.x_obsts]
        self.y_obsts = [y * path_unit for y in self.y_obsts]
        # start - target
        self.x_starts = [x * path_unit for x in self.x_starts]
        self.y_starts = [y * path_unit for y in self.y_starts]
        self.x_targets = [x * path_unit for x in self.x_targets]
        self.y_targets = [y * path_unit for y in self.y_targets]

    def map_0(self, robot_n: int):
        # area
        lim = 13
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim
        self.map_id = 1

        # obstacles
        xc1 = [3, 3, 5, 5, 7, 7, 9, 9, 11, 11]
        yc1 = [3, 5, 3, 5, 3, 5, 3, 5, 3, 5]
        xc2 = xc1*2
        yc2 = yc1 + [y+6 for y in yc1]
        self.x_obsts = xc2
        self.y_obsts = yc2
        self.n_obsts = len(self.x_obsts)

        # robots
        self.n_robots = robot_n
        x_starts = [1, 12, 10.3, 4, 12, 1, 6, 1, 10, 1, 8, 3, 4, 6, 11]
        y_starts = [3, 10, 1.2, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5, 4, 4]
        x_targets = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 6, 8, 4]
        y_targets = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 3, 12, 11, 7, 11]
        self.headings = [0.0 for i in range(self.n_robots)]
        self.x_starts = [x_starts[i] for i in range(self.n_robots)]
        self.y_starts = [y_starts[i] for i in range(self.n_robots)]
        self.x_targets = [x_targets[i] for i in range(self.n_robots)]
        self.y_targets = [y_targets[i] for i in range(self.n_robots)]

    def warehouse_1(self, robot_n=1):
        # area
        lim = 13
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = 42
        self.y_max = 18

        # obstacles
        xc1 = [3, 4, 5, 6, 7, 8]
        yc1 = [2 for i in xc1] + [3 for i in xc1]
        xc1 = xc1*2
        xc2 = xc1 + [x+8 for x in xc1] + [x+16 for x in xc1] + [x+24 for x in xc1] + [x+32 for x in xc1]
        yc2 = yc1*5
        xc3 = xc2*3
        yc3 = [y+4 for y in yc2] + [y+8 for y in yc2] + [y+12 for y in yc2]
        self.x_obsts = xc2 + xc3
        self.y_obsts = yc2 + yc3

        # robots
        self.n_robots = robot_n

        self.headings = [0.0]
        self.x_starts = [1]
        self.y_starts = [1]
        self.x_targets = [1]
        self.y_targets = [10]

    def collide(self, n_robots=2):
        # area
        lim = 7
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim
        self.map_id = 1

        # obstacles
        # xc1 = [3, 3, 5, 5, 7, 7, 9, 9, 11, 11]
        # yc1 = [3, 5, 3, 5, 3, 5, 3, 5, 3, 5]
        # xc2 = xc1*2
        # yc2 = yc1 + [y+6 for y in yc1]
        self.x_obsts = []  # xc2
        self.y_obsts = []  # yc2
        self.n_obsts = 0

        # robots
        self.n_robots = n_robots
        # # 1
        # x_starts = [2, 5]
        # y_starts = [4, 4]
        # x_targets = [6, 1]
        # y_targets = [4, 4]
        # self.headings = [0.0 , 3.14]
        # 2
        x_starts = [2, 2]
        y_starts = [2, 5]
        x_targets = [5, 5]
        y_targets = [5, 2]
        self.headings = [0.0, 0.0]
        # # 3
        # x_starts = [0, 0]
        # y_starts = [1, 0]
        # x_targets = [5, 3]
        # y_targets = [1, 2]
        # self.headings = [0.0 , 0.0]
        n_robots = 2
        self.rids = list(range(1, n_robots+1))
        self.x_starts = [x_starts[i] for i in range(n_robots)]
        self.y_starts = [y_starts[i] for i in range(n_robots)]
        self.x_targets = [x_targets[i] for i in range(n_robots)]
        self.y_targets = [y_targets[i] for i in range(n_robots)]

    def random_map_1(self):
        #
        obst_n = 20
        robots_n = 10
        self.n_robots = robots_n

        # area
        lim = 11
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim

        # x-y random
        xy = []
        xx = np.arange(self.x_min, self.x_max, 1.0).tolist()
        yy = np.arange(self.y_min, self.y_max, 1.0).tolist()
        for y in yy:
            for x in xx:
                xy.append([x, y])

        r_xy = xy[:]
        random.shuffle(r_xy)

        # obstacles
        obst_i = r_xy[0:obst_n]
        robots_s_i = r_xy[obst_n:obst_n+robots_n]
        robots_g_i = r_xy[obst_n+robots_n:obst_n+2*robots_n]
        self.x_obsts = [o[0] for o in obst_i]
        self.y_obsts = [o[1] for o in obst_i]

        # robots

        self.headings = [0.0 for i in range(self.n_robots)]
        self.x_starts = [s[0] for s in robots_s_i]
        self.y_starts = [s[1] for s in robots_s_i]
        self.x_targets = [t[0] for t in robots_g_i]
        self.y_targets = [t[1] for t in robots_g_i]

    def random_map_2(self, ind=1):
        #
        self.map_id = ind
        obst_n = 22
        robots_n = 8
        self.n_robots = robots_n

        # area
        lim = 9
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim

        # obstacles
        xo = np.random.uniform(self.x_min, self.x_max)
        yo = np.random.uniform(self.y_min, self.y_max)
        obst_x = [xo]
        obst_y = [yo]
        while len(obst_x) < obst_n:
            dist = 0
            while dist < 0.5:
                xo = np.random.uniform(self.x_min, self.x_max)
                yo = np.random.uniform(self.y_min, self.y_max)
                dist = min([cal_distance(x, y, xo, yo) for (x, y) in zip(obst_x, obst_y)])
            obst_x.append(xo)
            obst_y.append(yo)

        # robots start and target points
        all_x = obst_x[:]
        all_y = obst_y[:]
        x_rs = []
        y_rs = []
        while len(x_rs) < robots_n:
            dist = 0
            while dist < 1:
                x_starts = np.random.uniform(self.x_min, self.x_max)
                y_starts = np.random.uniform(self.y_min, self.y_max)
                dist = min([cal_distance(x, y, x_starts, y_starts) for (x, y) in zip(all_x, all_y)])
            all_x.append(x_starts)
            all_y.append(y_starts)
            x_rs.append(x_starts)
            y_rs.append(y_starts)

        ii = 0
        x_rt = []
        y_rt = []
        while len(x_rt) < robots_n:
            dist = 0
            while True:
                x_targets = np.random.uniform(self.x_min, self.x_max)
                y_targets = np.random.uniform(self.y_min, self.y_max)
                dist = min([cal_distance(x, y, x_targets, y_targets) for (x, y) in zip(all_x, all_y)])

                if dist > 1:
                    dist_st = cal_distance(x_rs[ii], y_rs[ii], x_targets, y_targets)
                    if dist_st > 5:
                        ii += 1
                        x_rt.append(x_targets)
                        y_rt.append(y_targets)
                        all_x.append(x_targets)
                        all_y.append(y_targets)
                        break
        x_s = x_rs
        y_s = y_rs
        x_t = x_rt
        y_t = y_rt

        # obstacles
        self.x_obsts = obst_x
        self.y_obsts = obst_y
        self.n_obsts = len(obst_x)

        # robots

        self.headings = [0.0 for i in range(self.n_robots)]
        self.x_starts = x_s
        self.y_starts = y_s
        self.x_targets = x_t
        self.y_targets = y_t

        # dave data as JSON
        self.save_object_attributes(ind)

    def save_object_attributes(self, ind=1):

        # file name
        ind = str(ind)
        no = 'o'+str(len(self.x_obsts))+'_map'+ind+'.json'
        map_name = 'maps/'+no
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)

        # Convert object attributes to a dictionary
        obj_dict = vars(self)

        # Save the dictionary as a JSON file
        with open(filename, "w") as file:
            json.dump(obj_dict, file)

    def from_json_file(self, n=1):

        # file name
        ind = str(n)
        no = 'o15'+'_map'+ind+'.json'
        map_name = 'maps/'+no
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)

        # load the JSON file
        with open(filename, "r") as file:
            data = json.load(file)

        # map index
        self.map_id = data['map_id']

        # area
        lim = data['lim']
        self.lim = lim
        self.x_min = data['x_min']
        self.y_min = data['y_min']
        self.x_max = data['lim']
        self.y_max = data['lim']

        # obstacles
        self.n_obsts = len(data['x_obsts'])
        self.x_obsts = data['x_obsts']
        self.y_obsts = data['y_obsts']

        # robots
        self.n_robots = data['n_robots']
        self.rids = data['rids']
        self.headings = data['headings']
        self.x_starts = data['x_starts']
        self.y_starts = data['y_starts']
        self.x_targets = data['x_targets']
        self.y_targets = data['y_targets']

        # modify obstacles
        d_prec = 0.4/self.path_unit
        new_obst_x = []
        new_obst_y = []
        for i, [x, y] in enumerate(zip(self.x_obsts, self.y_obsts)):
            for j in range(i+1, len(self.x_obsts)):
                dist = cal_distance(x, y, self.x_obsts[j], self.y_obsts[j])
                if d_prec < dist < 1.65*d_prec:
                    new_obst_x.append((self.x_obsts[j]+x)/2)
                    new_obst_y.append((self.y_obsts[j]+y)/2)

        self.x_obsts.extend(new_obst_x)
        self.y_obsts.extend(new_obst_y)

    def modify_obst(self):
        # modify obstacles
        d_prec = 0.4/self.path_unit
        new_obst_x = []
        new_obst_y = []
        for i, [x, y] in enumerate(zip(self.x_obsts, self.y_obsts)):
            for j in range(i+1, len(self.x_obsts)):
                dist = cal_distance(x, y, self.x_obsts[j], self.y_obsts[j])
                if dist < 1.99*d_prec and dist > d_prec:
                    xxm = (self.x_obsts[j]+x)/2.0
                    yym = (self.y_obsts[j]+y)/2.0
                    new_obst_x.append((x+xxm)/2.0)
                    new_obst_y.append((y+yym)/2.0)
                    new_obst_x.append((self.x_obsts[j]+xxm)/2.0)
                    new_obst_y.append((self.y_obsts[j]+yym)/2.0)
                    new_obst_x.append(xxm)
                    new_obst_y.append(yym)
        self.x_obsts.extend(new_obst_x)
        self.y_obsts.extend(new_obst_y)

    def obstacles2(self, n_robot=2):

        # area
        lim = 7
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim
        self.map_id = 1

        # obstacles
        xc2 = [3.1, 3.9]
        yc2 = [3.5, 3.5]
        self.x_obsts = xc2
        self.y_obsts = yc2
        self.n_obsts = len(self.x_obsts)
        # self.modify_obst()

        # robots
        # # 1
        # x_starts = [3.5] #[3.5, 3.5]
        # y_starts = [1.0] #[1.0, 3.5]
        # x_targets = [3.5] #[3.5, 3.5]
        # y_targets = [6.0] #[6.0, 3.5]
        # self.n_robots = 1
        # self.headings = [np.pi/2] #[np.pi/2, -np.pi/2]
        x_starts = [3.5, 3.5]
        y_starts = [2.3, 3.5]
        x_targets = [3.5, 3.5]
        y_targets = [6.0, 3.5]
        self.n_robots = n_robot
        self.headings = [np.pi/2, -np.pi/2]
        n_robots = self.n_robots
        self.rids = list(range(1, n_robots+1))
        self.x_starts = [x_starts[i] for i in range(n_robots)]
        self.y_starts = [y_starts[i] for i in range(n_robots)]
        self.x_targets = [x_targets[i] for i in range(n_robots)]
        self.y_targets = [y_targets[i] for i in range(n_robots)]

    def obstacles1(self, n_robot=1):

        # area
        lim = 7
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim
        self.map_id = 1

        # obstacles
        xc2 = [3.5]
        yc2 = [3.5]
        self.x_obsts = xc2
        self.y_obsts = yc2
        self.n_obsts = len(self.x_obsts)
        # self.modify_obst()

        # robots
        self.n_robots = n_robot
        # 1
        x_starts = [3.5]  # [3.5, 3.5]
        y_starts = [2.3]  # [1.0, 3.5] //////
        x_targets = [3.5]  # [3.5, 3.5]
        y_targets = [6.0]  # [6.0, 3.5]
        self.headings = [np.pi/2]  # [np.pi/2, -np.pi/2]
        n_robots = self.n_robots
        self.rids = list(range(1, n_robots+1))
        self.x_starts = [x_starts[i] for i in range(n_robots)]
        self.y_starts = [y_starts[i] for i in range(n_robots)]
        self.x_targets = [x_targets[i] for i in range(n_robots)]
        self.y_targets = [y_targets[i] for i in range(n_robots)]
