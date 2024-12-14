#! /usr/bin/env python

import json
import rospkg
import numpy as np

class Results:
    def __init__(self, paths, times, path_unit, test_s, save_path):
        
        # input data
        self.paths = paths
        self.times = times

        # preallocation
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
        
        # settings
        self.path_unit = 1
        self.test_name = "res_" + test_s + ".json"

        # calculate len, smoothness and time for each robot
        for k, v in paths.items():
            self.headings.append(round(self.cal_angle(v), 2))
            self.lens.append(round(self.cal_len(v), 2))
            self.times.append(round(times[k][-1], 2))
            # self.angles.append(round(angles[k], 2))
            # self.angles2.append(angles[k])
        
        self.total_len = round(sum(self.lens), 2)
        self.mean_len = round(self.total_len/len(self.lens), 2)
        self.total_time = round(sum(self.times),2)
        self.operation_time = max(self.times)
        self.total_headings = [round(sum(self.headings), 2)]
        self.total_headings.append(round(180*self.total_headings[0]/np.pi, 2)) 
        # self.total_angle = [round(sum(self.angles2), 2)]
        # self.total_angle.append(round(180*self.total_angle[0]/np.pi, 2)) 

        # final data
        final_data = {"mean_len": self.mean_len,"operation_time":self.operation_time,
                        "lens":self.lens, "times": self.times, "total_time": self.total_time, 
                        "total_len": self.total_len}

        heading_data = {"headings":self.headings, "total_heading":self.total_headings}      
        n_robots = {"n_robots": len(paths)}
        
        # # save data JSON
        all_data = [final_data, heading_data, n_robots]
        with open(save_path, "w") as outfile:
            json.dump(all_data, outfile, indent=2)
            outfile.write("\n")

        # with open(save_path, "a") as outfile:
        #     json.dump(heading_data, outfile, indent=2)
        #     outfile.write("\n")
        #     json.dump(n_robots, outfile, indent=2)
        
        print("==================================")
        print("operation_time", self.operation_time)
        print("==================================")



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
            thetas.append(theta)

        dif_theta = [abs(self.angle_diff(thetas[i+1], thetas[i])) for i in range(len(thetas)-1)]
        dif_theta = sum(dif_theta)
        return dif_theta

    def angle_diff(self, a1, a2):
        da = a1 - a2
        return np.arctan2(np.sin(da), np.cos(da))
