#! /usr/bin/env python

""" Generate MRPP APF Results """

import json
import numpy as np
from parameters import Params
from mrapf_classes import AllPlannersData


class Results:

    def __init__(self, planners_data: AllPlannersData, result_path: str, params: Params):

        # settings
        self.path_unit = 1
        self.data: AllPlannersData = planners_data
        self.n = planners_data.n
        self.params = params
        total_success = planners_data.n_reached == planners_data.n

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
        all_durations = np.array([t for i, t in self.data.all_durations.items()])
        all_lengths = np.round(all_lengths, 2)
        all_d_headings = np.round(all_d_headings, 2)
        all_durations = np.round(all_durations, 2)

        #  total
        total_length = np.round(np.sum(all_lengths), 2)
        mean_length = np.round(total_length/self.n, 2)
        total_time = np.round(np.sum(all_durations), 2)
        operation_time = np.max(all_durations)
        total_h = np.sum(all_d_headings)
        total_headings = np.array([total_h, np.rad2deg(total_h)])
        total_headings = np.round(total_headings, 2)

        # max_steps
        max_steps = max(self.data.all_steps)

        # final data
        final_data = {
            "map_id": self.params.map_id,
            "nD": self.params.simD,
            "method": self.params.method,
            "n_robots": self.n,
            "n_reached": planners_data.n_reached,
            "success": total_success,
            "operation_time": operation_time,
            "max_steps": max_steps,
            "mean_length": mean_length,
            "total_length": total_length,
            "total_heading": total_headings.tolist(),
            "all_lengths": all_lengths.tolist(),
            "all_durations": all_durations.tolist(),
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
