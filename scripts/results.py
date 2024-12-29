""" MRPP APF Results """
#! /usr/bin/env python

import json
import numpy as np
from mrapf_classes import AllPlannersData


class Results:
    planners_data: AllPlannersData
    path_unit: float

    def __init__(self, planners_data: AllPlannersData, result_path: str):

        # settings
        self.path_unit = 1
        self.data = planners_data
        self.n = planners_data.n

        #
        all_lengths = []
        all_d_headings = []
        for i in range(self.n):
            dx = np.diff(self.data.all_x[i]) * self.path_unit
            dy = np.diff(self.data.all_y[i]) * self.path_unit
            l = np.sum(np.sqrt(dx**2, dy**2))
            h = np.arctan2(dy, dx)
            dh = np.diff(h)
            dh = np.sum([abs(self.angle_diff(th)) for th in h])
            dh = np.round(dh, 2)
            all_lengths.append(l)
            all_d_headings.append(dh)
        #
        all_times = np.array([t for i, t in self.data.all_times.items()])
        all_lengths = np.round(all_lengths, 2)
        all_d_headings = np.round(all_d_headings, 2)
        all_times = np.round(all_times, 2)

        #  total
        total_length = np.round(np.sum(all_lengths), 2)
        mean_length = np.round(total_length/self.n, 2)
        total_time = np.round(np.sum(all_times), 2)
        operation_time = np.max(all_times)
        total_h = np.sum(all_d_headings)
        total_headings = np.array([total_h, np.rad2deg(total_h)])
        total_headings = np.round(total_headings, 2)

        # max_steps
        max_steps = max(self.data.all_steps)

        # final data
        final_data = {"n_robots": self.n,
                      "max_steps": max_steps,
                      "mean_length": mean_length,
                      "total_length": total_length,
                      "operation_time": operation_time,
                      "total_heading": total_headings.tolist(),
                      "all_lengths": all_lengths.tolist(),
                      "all_times": all_times.tolist(),
                      "total_time": total_time,
                      "headings": all_d_headings.tolist()
                      }

        # save data JSON
        with open(result_path + "res.json", "w") as outfile:
            json.dump(final_data, outfile, indent=2)
            outfile.write("\n")

        print("==================================")
        print("[Results]: operation_time:", operation_time)
        print("==================================")

    def angle_diff(self, da):
        return np.arctan2(np.sin(da), np.cos(da))
