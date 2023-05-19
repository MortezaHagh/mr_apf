import os
import json
import random
import rospkg
import numpy as np


class ModelInputs():
    def __init__(self, map_id=1, path_unit=1.0, robot_count = 1):
        print("Inputs for creating model")

        if map_id == 1:
            # self.map_0(robot_count)
            self.collide()
            # self.random_map_2(robot_count)
            # self.from_json_file(robot_count, path_unit)

        self.apply_path_unit(path_unit)

    def apply_path_unit(self, path_unit):
        self.lim = self.lim * path_unit
        self.x_min = self.x_min * path_unit
        self.y_min = self.y_min * path_unit
        self.x_max = self.x_max * path_unit
        self.y_max = self.y_max * path_unit

        self.x_obst = [x * path_unit for x in self.x_obst]
        self.y_obst = [y * path_unit for y in self.y_obst]

        self.xs = [x * path_unit for x in self.xs]
        self.ys = [y * path_unit for y in self.ys]
        self.xt = [x * path_unit for x in self.xt]
        self.yt = [y * path_unit for y in self.yt]



    def map_0(self, robot_n):
        # area
        lim = 13
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim

        # obstacles
        xc1 = [3, 3, 5, 5, 7, 7, 9, 9, 11, 11]
        yc1 = [3, 5, 3, 5, 3, 5, 3, 5, 3, 5]

        xc2 = xc1*2
        yc2 = yc1 + [y+6 for y in yc1]
    
        self.x_obst = []
        self.y_obst = []

        # self.x_obst = xc2
        # self.y_obst = yc2

        # robots
        self.robot_count = robot_n

        # robots
        xs = [1, 12, 10.3, 4, 12, 1, 6, 1, 10,1, 8, 3, 4, 6, 11] 
        ys = [3, 10, 1.2, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5, 4, 4]
        xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 6, 8, 4]
        yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 3, 12, 11, 7, 11]

        # xs = [1, 12, 10.3, 4, 12, 1, 6, 1, 10,1, 8, 3, 4, 6, 11] 
        # ys = [3, 10, 1.2, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5, 4, 4]
        # xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 6, 8.5, 4]
        # yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 7, 3, 12, 11, 7, 12]

        self.ids = list(range(1, self.robot_count+1))
        self.heading = [0.0 for i in range(self.robot_count)]
        self.xs = [xs[i] for i in range(self.robot_count)]
        self.ys = [ys[i] for i in range(self.robot_count)]
        self.xt = [xt[i] for i in range(self.robot_count)]
        self.yt = [yt[i] for i in range(self.robot_count)]
    

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
        yc3 =  [y+4 for y in yc2] + [y+8 for y in yc2] + [y+12 for y in yc2]

        self.x_obst = xc2 + xc3
        self.y_obst = yc2 + yc3

        # robots
        robot_n = robot_n
        self.robot_count = robot_n

        # robots
        self.ids = list(range(1,self.robot_count+1))
        self.heading = [0.0]
        self.xs = [1]
        self.ys = [1]
        self.xt = [1]
        self.yt = [10]


    def collide(self, robot_n=2):
        
        # area
        lim = 13
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim
        self.map_ind = 1
        
        # obstacles
        xc1 = [3, 3, 5, 5, 7, 7, 9, 9, 11, 11]
        yc1 = [3, 5, 3, 5, 3, 5, 3, 5, 3, 5]

        xc2 = xc1*2
        yc2 = yc1 + [y+6 for y in yc1]

        self.x_obst = [] # xc2
        self.y_obst = [] # yc2

        # robots
        self.robot_count = 2

        # robots

        # 1
        xs = [3, 7] 
        ys = [0, 0]
        xt = [7, 3]
        yt = [0, 0]
        # xs = [3, 7] 
        # ys = [0, 0.1]
        # xt = [7, 3]
        # yt = [0, 0.0]
        self.heading = [0.0 , 3.14]

        # # 2
        # xs = [0, 0] 
        # ys = [0, 2]
        # xt = [4, 4]
        # yt = [2, 0]
        # self.heading = [0.0 , 0.0]
        
        # # 3
        # xs = [0, 0] 
        # ys = [1, 0]
        # xt = [5, 3]
        # yt = [1, 2]
        # self.heading = [0.0 , 0.0]

        robot_count = 2
        self.ids = list(range(1,robot_count+1))
        self.xs = [xs[i] for i in range(robot_count)]
        self.ys = [ys[i] for i in range(robot_count)]
        self.xt = [xt[i] for i in range(robot_count)]
        self.yt = [yt[i] for i in range(robot_count)]


    def random_map(self):

        obst_n = 20
        robots_n = 10
        self.robot_count = robots_n

        # area
        lim = 11
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim

        xx = np.arange(self.x_min, self.x_max, 1.0).tolist()
        yy = np.arange(self.y_min, self.y_max, 1.0).tolist()

        m = []
        for y in yy:
            for x in xx:
                m.append([x, y])
        
        r_m = m[:]
        random.shuffle(r_m)

        obst_i = r_m[0:obst_n]
        robots_s_i = r_m[obst_n:obst_n+robots_n]
        robots_g_i = r_m[obst_n+robots_n:obst_n+2*robots_n]

        self.x_obst = [o[0] for o in obst_i]
        self.y_obst = [o[1] for o in obst_i]
        
        self.ids = list(range(1, self.robot_count+1))
        self.heading = [0.0 for i in range(self.robot_count)]
        self.xs = [s[0] for s in robots_s_i]
        self.ys = [s[1] for s in robots_s_i]
        self.xt = [t[0] for t in robots_g_i]
        self.yt = [t[1] for t in robots_g_i]

        # print(self.robot_count)


    def random_map_2(self, ind=1):
        
        self.map_ind = ind
        obst_n = 22
        robots_n = 8
        self.robot_count = robots_n

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
        while len(obst_x)<obst_n:
            dist = 0
            while dist < 0.5:
                xo = np.random.uniform(self.x_min, self.x_max)
                yo = np.random.uniform(self.y_min, self.y_max)
                dist = min([self.distance(x, y, xo, yo) for (x, y) in zip(obst_x, obst_y)])
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
                xs = np.random.uniform(self.x_min, self.x_max)
                ys = np.random.uniform(self.y_min, self.y_max)
                dist = min([self.distance(x, y, xs, ys) for (x, y) in zip(all_x, all_y)])
            all_x.append(xs)
            all_y.append(ys)
            x_rs.append(xs)
            y_rs.append(ys)

        ii = 0
        x_rt = []
        y_rt = []
        while len(x_rt) < robots_n:
            dist = 0
            while True:
                xt = np.random.uniform(self.x_min, self.x_max)
                yt = np.random.uniform(self.y_min, self.y_max)
                dist = min([self.distance(x, y, xt, yt) for (x, y) in zip(all_x, all_y)])

                if dist >1:
                    dist_st = self.distance(x_rs[ii], y_rs[ii], xt, yt)
                    if dist_st > 5:
                        ii+=1
                        x_rt.append(xt)
                        y_rt.append(yt)
                        all_x.append(xt)
                        all_y.append(yt)
                        break
        x_s = x_rs
        y_s = y_rs
        x_t = x_rt
        y_t = y_rt

        # obstacles
        self.x_obst = obst_x
        self.y_obst = obst_y
        self.obst_count_orig = len(obst_x)

        # robots
        self.ids = list(range(1, self.robot_count+1))
        self.heading = [0.0 for i in range(self.robot_count)]
        self.xs = x_s
        self.ys = y_s
        self.xt = x_t
        self.yt = y_t

        # dave data as JSON
        self.save_object_attributes(ind)


    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)
    

    def save_object_attributes(self, ind=1):
        
        # file name
        ind = str(ind)
        no = 'o'+str(len(self.x_obst))+'_map'+ind+'.json'
        map_name = 'maps/'+no
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)

        # Convert object attributes to a dictionary
        obj_dict = vars(self)

        # Save the dictionary as a JSON file
        with open(filename, "w") as file:
            json.dump(obj_dict, file)


    def from_json_file(self, n=1, path_unit=1):
        
        # file name
        ind = str(n)
        no = 'o22'+'_map'+ind+'.json'
        map_name = 'maps/'+no
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        filename = os.path.join(pkg_path, map_name)

        # load the JSON file
        with open(filename, "r") as file:
            data = json.load(file)

        # map index
        self.map_ind = data['map_ind']

        # area
        lim = data['lim']
        self.lim = lim
        self.x_min = data['x_min']
        self.y_min = data['y_min']
        self.x_max = data['lim']
        self.y_max = data['lim']
        
        # obstacles
        self.obst_count_orig = len(data['x_obst'])
        self.x_obst = data['x_obst']
        self.y_obst = data['y_obst']

        # robots
        self.robot_count = data['robot_count']

        # robots
        self.ids = data['ids']
        self.heading = data['heading']
        self.xs = data['xs']
        self.ys = data['ys']
        self.xt = data['xt']
        self.yt = data['yt']

        # modify obstacles
        obst_prec_d = 0.4/path_unit
        new_obst_x = []
        new_obst_y = []
        for i in range(len(self.x_obst)):
            x = self.x_obst[i]
            y = self.y_obst[i]
            for j in range(i+1,len(self.x_obst)):
                dist = self.distance(x, y, self.x_obst[j], self.y_obst[j])
                if dist<1.65*obst_prec_d and dist>obst_prec_d:
                    new_obst_x.append((self.x_obst[j]+x)/2)
                    new_obst_y.append((self.y_obst[j]+y)/2)
        
        self.x_obst.extend(new_obst_x)
        self.y_obst.extend(new_obst_y)


# ----------------------------------------------------------------

if __name__=="__main__":
    mi = ModelInputs()
    mi.random_map()