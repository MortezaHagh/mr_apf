

class ModelInputs():
    def __init__(self, map_id=-1, path_unit=1.0, robot_count = 1):
        print("Inputs for creating model")

        if map_id == -1:
            self.map_0()

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



    def map_0(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 1

        # robots
        xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8, 3, 4, 6] 
        ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5, 4]
        xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 5, 8]
        yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1, 13, 7, 7]
        
        robot_count = 1
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0 for i in range(robot_count)]
        self.xs = [xs[i] for i in range(robot_count)]
        self.ys = [ys[i] for i in range(robot_count)]
        self.xt = [xt[i] for i in range(robot_count)]
        self.yt = [yt[i] for i in range(robot_count)]
    
    def map_4(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 4

        # robots
        robot_count = 4
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4]
        self.ys = [3, 10, 1, 12]
        self.xt = [10, 4, 4, 10]
        self.yt = [10, 4, 10, 4]

    def map_5(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 5

        # robots
        robot_count = 5
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12]
        self.ys = [3, 10, 1, 12, 6]
        self.xt = [10, 4, 4, 10, 2]
        self.yt = [10, 4, 10, 4, 10]

    def map_6(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 6

        # robots
        robot_count = 6
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1]
        self.ys = [3, 10, 1, 12, 6, 8]
        self.xt = [10, 4, 4, 10, 3, 10]
        self.yt = [10, 4, 10, 4, 8, 7]

    def map_7(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 7

        # robots
        robot_count = 7
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6]
        self.ys = [3, 10, 1, 12, 6, 8, 10]
        self.xt = [10, 4, 4, 10, 3, 10, 6]
        self.yt = [10, 4, 10, 4, 8, 7, 5]

    def map_8(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 8

        # robots
        robot_count = 8
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7]

    def map_9(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 9

        # robots
        robot_count = 9
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7]

    def map_10(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 10

        # robots
        robot_count = 10
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8]

    def map_11(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 11

        # robots
        robot_count = 11
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1]

    def map_12(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 12

        # robots
        robot_count = 12
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0 for i in range(robot_count+1)]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8, 3]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11, 1]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1, 13]

    def map_13(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 13

        # robots
        robot_count = 13
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0 for i in range(robot_count)]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8, 3, 4]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 5]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1, 13, 7]

    def map_14(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 14

        # robots
        robot_count = 14
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0 for i in range(robot_count)]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8, 3, 4, 6]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11, 1, 5, 4]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10, 5, 8]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1, 13, 7, 7]

    def map_test_visualize(self):
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

        self.x_obst = xc2
        self.y_obst = yc2

        # robots
        self.robot_count = 4

        # robots
        robot_count = 4
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0]
        self.xs = [0, 12, 10, 3]
        self.ys = [7, 7, 1, 12]
        self.xt = [10, 4, 4, 10]
        self.yt = [10, 4, 10, 4]

    def warehouse_1(self):
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
        self.robot_count = 1

        # robots
        robot_count = 1
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0]
        self.xs = [1]
        self.ys = [1]
        self.xt = [1]
        self.yt = [10]

    def warehouse_2(self):
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
        self.robot_count = 20

        # robots
        robot_count = 20
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0]
        self.xs = []
        self.ys = []
        self.xt = []
        self.yt = []
