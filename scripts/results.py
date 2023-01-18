#! /usr/bin/env python

import json
import rospkg
import numpy as np

class Results:
    def __init__(self, paths, times, path_unit, test_id):
        self.paths = paths
        self.times = times
        self.lens = []
        self.times = []
        # self.angles = []
        # self.angles2 = []
        self.total_len = 0
        self.total_time = 0
        # self.total_angle = 0
        self.operation_time = 0
        self.headings = []
        self.total_headings = 0
        self.path_unit = 1
        self.test_name = "res_" + str(test_id) + ".json"

        #
        for k, v in paths.items():
            self.lens.append(round(self.cal_len(v), 2))
            self.times.append(round(times[k][-1], 2))
            # self.angles.append(round(angles[k], 2))
            # self.angles2.append(angles[k])
            self.headings.append(round(self.cal_angle(v), 2))
        
        self.total_len = round(sum(self.lens), 2)
        self.mean_len = round(self.total_len/len(self.lens), 2)
        self.total_time = round(sum(self.times),2)
        self.operation_time = max(self.times)
        # self.total_angle = [round(sum(self.angles2), 2)]
        # self.total_angle.append(round(180*self.total_angle[0]/np.pi, 2)) 
        self.total_headings = [round(sum(self.headings), 2)]
        self.total_headings.append(round(180*self.total_headings[0]/np.pi, 2)) 

        # 
        final_data = {"mean_len": self.mean_len,"operation_time":self.operation_time,
                        "lens":self.lens, "times": self.times, "total_time": self.total_time, 
                        "total_len": self.total_len}

        # final_data = {"lens":self.lens, "times": self.times, "total_time": self.total_time, 
        #               "total_len": self.total_len, "operation_time":self.operation_time}

        hwading_data = {"headings":self.headings, "total_heading":self.total_headings}      
        robot_count = {"robot_count": len(paths)}
        
        #
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('apf')
        save_path = pkg_path + '/result_apf/test_paper3/' + self.test_name
        with open(save_path, "w") as outfile:
            json.dump(final_data, outfile, indent=2)
            outfile.write("\n")
        with open(save_path, "a") as outfile:
            json.dump(hwading_data, outfile, indent=2)
            outfile.write("\n")
            json.dump(robot_count, outfile, indent=2)


    def cal_len(self, p):
        path_len = 0
        for i in range(len(p[0])-1):
            x1 = p[0][i]*self.path_unit
            y1 = p[1][i]*self.path_unit
            x2 = p[0][i+1]*self.path_unit
            y2 = p[1][i+1]*self.path_unit
            path_len = path_len + self.distance(x1, x2, y1, y2)
        return path_len
    

    def distance(self, x1, x2, y1, y2):
        return round(np.sqrt((x1-x2)**2 + (y1-y2)**2), 2)

    def cal_angle(self, p):
        thetas = [0]
        n = 1
        for i in range(len(p[0])-1):
            x1 = p[0][i*n]*self.path_unit
            y1 = p[1][i*n]*self.path_unit
            x2 = p[0][i+1]*self.path_unit  #[(i+1)*n-1]
            y2 = p[1][i+1]*self.path_unit  #[(i+1)*n-1]
            dx = x2-x1
            dy = y2-y1
            theta = round(np.arctan2(dy, dx),2)
            if theta<0:
                theta = theta + 2*np.pi
            thetas.append(theta)
        dif_theta = [abs(thetas[i+1]-thetas[i]) for i in range(len(thetas)-1)]
        dif_theta = sum(dif_theta)
        return dif_theta

