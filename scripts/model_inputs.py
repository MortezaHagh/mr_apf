

class ModelInputs():
    def __init__(self, map_id=-1):
        print("Inputs for creating model")

        if map_id == 1:
            self.map1()
        elif map_id == -1:
            self.map1()


    def map0(self):

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
        self.xt = [10]
        self.yt = [10]


    def map1(self):
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
        self.ids = list(range(1, self.robot_count+1))
    
    
    def map_now(self):
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
        self.heading = [0.0 for i in range(robot_count)]
        self.xs = [1, 12, 10, 4, 12, 1, 6, 1, 10,1, 8, 3]
        self.ys = [3, 10, 1, 12, 6, 8, 10, 6, 9, 12, 11, 1]
        self.xt = [10, 4, 4, 10, 3, 10, 6, 7, 1, 5, 2, 10]
        self.yt = [10, 4, 10, 4, 8, 7, 5, 7, 7, 8, 1, 13]


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
        self.robot_count = 0

        # robots
        robot_count = 7
        self.ids = list(range(1,robot_count+1))
        self.heading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.xs = [1, 12, 10, 4, 12, 1, 6]
        self.ys = [3, 10, 1, 12, 6, 8, 10]
        self.xt = [10, 4, 4, 10, 3, 10, 6]
        self.yt = [10, 4, 10, 4, 8, 7, 5]
