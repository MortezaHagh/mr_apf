

class ModelInputs():
    def __init__(self, map_id=4):
        print("Inputs for creating model")

        if map_id == 1:
            self.map1()
        elif map_id == 2:
            self.map2()
        elif map_id == 3:
            self.map3()
        elif map_id == 4:
            self.map4()
        elif map_id == 5:
            self.map5()

    def map1(self):

        # area
        lim = 26
        self.lim = lim
        self.x_min = 0
        self.y_min = 0
        self.x_max = lim
        self.y_max = lim

        # obstacles
        xc1 = [3, 3, 3, 5, 5, 5, 7, 7, 7, 9, 9, 9, 11, 11, 11]
        yc1 = [3, 4, 5, 3, 4, 5, 3, 4, 5, 3, 4, 5, 3, 4, 5]

        xc2 = xc1*4
        yc2 = yc1 + [y+6 for y in yc1] + \
            [y+11 for y in yc1] + [y+16 for y in yc1]

        xc3 = [x+12 for x in xc2]
        yc3 = yc2

        self.x_obst = xc2 + xc3
        self.y_obst = yc2 + yc3

        # robots
        self.robot_count = 4
        self.heading = [0, 0, 0, 0]
        self.xs = [14, 11, 5, 1]
        self.ys = [11, 13, 7, 1]
        self.xt = [14, 20, 1, 12]
        self.yt = [17, 13, 12, 7]

    def map2(self):

        # area
        lim = 28
        self.lim = lim
        self.x_min = 1
        self.y_min = 1
        self.x_max = lim
        self.y_max = lim

        # obstacles
        xc1 = [3, 3, 3, 5, 5, 5, 7, 7, 7, 9, 9, 9, 11, 11, 11]
        yc1 = [3, 4, 5, 3, 4, 5, 3, 4, 5, 3, 4, 5, 3, 4, 5]

        xc1 = [x+1 for x in xc1]
        yc1 = [y+1 for y in yc1]

        xc2 = xc1*4
        yc2 = yc1 + [y+6 for y in yc1] + \
            [y+12 for y in yc1] + [y+18 for y in yc1]

        xc3 = [x+12 for x in xc2]
        yc3 = yc2

        self.x_obst = xc2 + xc3
        self.y_obst = yc2 + yc3

        # robots
        self.robot_count = 29
        self.heading = [0 for i in range(self.robot_count)]
        self.xs = [1, 25,  5, 20,  5, 12,  1,  2,  4,  1,  8, 17,  1, 15,
                   2, 24,  6, 21, 25, 10,  4, 10, 17, 21, 13, 17, 19, 22,  1]
        self.ys = [4,  4,  7,  1,  1,  1, 14, 19, 19, 16,  1,  1, 11,  4,
                   1,  1,  9,  4, 18,  1,  1,  9,  4,  9,  1,  9,  9,  1, 21]
        self.xt = [21,  4,  1,  5, 21, 12, 12, 24, 21, 25,  8, 17, 24,  6,
                   25,  1, 17,  6, 10, 10,  4, 24, 15, 17, 13, 15, 11,  6, 23]
        self.yt = [14, 14, 12, 23, 23, 25, 19, 19, 19, 16, 25, 24, 14, 19,
                   23, 24, 14, 14, 14, 19, 25,  9, 23, 19, 23, 19, 23, 25, 11]

    def map3(self):

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
        self.heading = [0.0]
        self.xs = [0]
        self.ys = [0]
        self.xt = [1]
        self.yt = [6]

    def map4(self):

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
        self.heading = [0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4]
        self.ys = [3, 10, 1, 12]
        self.xt = [10, 4, 4, 10]
        self.yt = [10, 4, 10, 4]
    
    def map5(self):

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
        self.heading = [0.0]
        self.xs = [1]
        self.ys = [3]
        self.xt = [4]
        self.yt = [4]
