#! /usr/bin/env python

import rospy
from mrapf_sc import Run
from parameters import Params


def main():
    # parameters
    params = Params(point=True)
    params.map_id = 1
    params.simD = '2D'
    params.method = 2
    params.nr = 2

    # number of robots
    n_robots = range(2, 15)
    for nr in n_robots:
        params.nr = nr
        run = Run(params_i=params)
        run.run()
        print(" \n ---------------------------------- \n")


if __name__ == "__main__":
    rospy.init_node("test_mrapf")
    main()
