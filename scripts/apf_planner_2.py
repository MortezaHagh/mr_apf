#! /usr/bin/env python

from typing import List, Tuple
import numpy as np
from geometry_msgs.msg import Pose2D
from parameters import Params
from create_model import MRSModel
from my_utils import cal_angle_diff, cal_distance
from mrapf_classes import PRobot, ApfRobot, AllRobotsData
from shapely.geometry import Point, shape, MultiPoint  # type: ignore # pylint: disable-all
# from shapely.geometry.polygon import Polygon


class APFPlanner:
    p: Params
    model: MRSModel
    robot: PRobot
    ard: AllRobotsData
    multi_robots_vis: List[ApfRobot]
    mp_bound: List[Tuple[float, float]]

    def __init__(self, model: MRSModel, robot: PRobot, params: Params):
        # data
        self.p = params
        self.model = model
        self.robot = robot
        self.map_data()

        #
        self.pose = Pose2D()
        self.ard = None
        self.multi_robots_vis = []
        self.mp_bound = []

        #
        self.v = 0
        self.w = 0

        #
        self.f_r = 0
        self.f_theta = 0
        self.phi = 0

        # compute vars
        self.ad_rg_h = None
        self.theta_rg = None
        self.goal_dist = None
        self.goal_theta = None
        #
        self.robot_f = []
        self.target_f = []
        self.obs_f = []

        # control vars
        self.reached = False
        self.stopped = False
        self.is_multi = False
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

        # control vars
        self.theta_rg = 0
        self.goal_theta = 0
        self.goal_dist = 1000

    def reset_vals(self):
        self.multi_robots_vis = []
        self.mp_bound = []
        #
        self.v = 0
        self.w = 0
        #
        self.f_r = 0
        self.f_theta = 0
        self.phi = 0
        #
        self.reached = False
        self.stopped = False
        self.is_multi = False
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

    def planner_move(self, pose: Pose2D, ard: AllRobotsData):
        # inputs - reset
        self.pose = pose
        self.ard = ard
        self.reset_vals()

        # check dist to goal
        if self.goal_dist < self.p.goal_dis_tresh:
            self.reached = True
            return

        # detect and group
        self.detect_group()

        # check stop_flag_multi
        if self.stop_flag_multi:
            print(f"[planner_move, {self.p.id}], stop_flag_multi")
            self.stopped = True
            return
        else:
            # calculate forces =================================================
            self.forces()

            # calculate velocities
            self.cal_vel()

            # check stop flags
            if self.stop_flag_obsts or self.stop_flag_robots:
                print("[planner_move, {self.p.id}], stop_flag_obsts or stop_flag_robots")
                self.v = 0
                # self.w = 0
                if abs(self.w) < (np.deg2rad(2)):
                    self.v = self.p.v_min_2

            # check stop_flag_full
            if self.stop_flag_full:
                print("[planner_move, {self.p.id}], stop_flag_full")
                self.stopped = True
                self.v = 0
                # self.w = 0

    def forces(self):
        # target
        f_r = 0
        f_theta = 0
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]
        # obstacles
        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]
        # robots
        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]
        # phi
        theta = np.arctan2(f_theta, f_r)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        # update class values
        self.f_r = f_r
        self.f_theta = f_theta
        self.phi = phi

        # self.pd.phis.append(phi)
        # self.pd.f_or.append(self.obs_f[0])
        # self.pd.f_ot.append(self.obs_f[1])
        # self.pd.f_tr.append(self.target_f[0])
        # self.pd.f_tt.append(self.target_f[1])

    def detect_group(self):

        #
        c_r = 2.5       # 2.5 3.0      #$ param 1
        c_radius = 2.5  # 2.5          #$ param 3
        is_goal_close = False

        # reset
        groups = []
        AD_h_rR = []
        new_robots = []
        multi_robots = []
        multi_robots_vis: List[ApfRobot] = []
        robots_inds = []
        robots_inds_f = {}
        self.new_robots = []

        # detect obsts
        self.detect_obsts()

        # get data
        ard = self.ard

        # is_goal_close
        goal_dist = cal_distance(self.pose.x, self.pose.y, self.goal_x, self.goal_y)
        if (goal_dist < (c_r*self.p.robot_start_d)):
            is_goal_close = True

        # get indices of robots in proximity circle
        for i in range(ard.nr):
            # rR
            dx = (ard.x[i] - self.pose.x)
            dy = (ard.y[i] - self.pose.y)
            d_rR = np.sqrt(dx**2 + dy**2)
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)
            ad_h_rR_abs = abs(ad_h_rR)
            ad_H_Rr = cal_angle_diff(ard.h[i], (theta_rR - np.pi))
            ad_H_Rr_abs = abs(ad_H_Rr)
            AD_h_rR.append(ad_h_rR)

            if (d_rR > (c_r * self.p.robot_start_d)):
                continue

            # if (not ard.reached[i]) or (d_rR < (1 * self.p.robot_start_d)):
            # if (d_rR < (1 * self.p.robot_start_d)) or ((not ard.reached[i]) or (ad_h_rR_abs < np.pi/2 or ad_H_Rr_abs < np.pi/2)):

            if (not ard.reached[i]) or (d_rR < (1 * self.p.robot_start_d)):
                robots_inds.append(i)

            # individual robots
            if (d_rR < (1 * self.p.robot_start_d)):
                nr = ApfRobot()
                nr.d = d_rR
                nr.x = ard.x[i]
                nr.y = ard.y[i]
                nr.H = ard.h[i]
                nr.h_rR = ad_h_rR
                nr.theta_rR = theta_rR
                nr.p = (
                    not (ard.reached[i] or ard.stopped[i])) and ard.pr[i] > 0
                nr.stop = ard.stopped[i]
                nr.reached = ard.reached[i]
                rc = self.p.robot_prec_d
                nr.r_prec = rc
                nr.r_half = 1.5 * rc
                nr.r_start = 2.0 * rc
                nr.z = 4 * self.p.fix_f * rc**4
                new_robots.append(nr)
                if ard.reached[i]:
                    XY = self.eval_obst(
                        ard.x[i], ard.y[i], self.p.robot_prec_d, d_rR)
                    for xy in XY:
                        dx_ = (xy[0] - self.pose.x)
                        dy_ = (xy[1] - self.pose.y)
                        d_rR_ = np.sqrt(dx_**2 + dy_**2)
                        theta_rR_ = np.arctan2(dy_, dx_)
                        ad_h_rR_ = cal_angle_diff(self.pose.theta, theta_rR_)
                        nnr = ApfRobot()
                        nnr.d = d_rR_
                        nnr.x = xy[0]
                        nnr.y = xy[1]
                        nnr.H = ard.h[i]
                        nnr.h_rR = ad_h_rR_
                        nnr.theta_rR = theta_rR_
                        nnr.p = ard.pr[i] > 0
                        nnr.stop = True
                        nnr.reached = True
                        nnr.r_prec = nr.r_prec/1.5  # $
                        nnr.r_half = 1.5 * nnr.r_prec
                        nnr.r_start = 2 * nnr.r_prec
                        nnr.z = 4 * self.p.fix_f * nnr.r_prec**4
                        new_robots.append(nnr)
                        multi_robots_vis.append(nnr)

        # if there is none robots in proximity
        if len(robots_inds) == 0:
            if len(multi_robots_vis) > 0:
                self.new_robots = new_robots
                self.multi_robots_vis = multi_robots_vis
            return

        # generate robots_inds_f (neighbor robots in proximity circle)
        if (not is_goal_close):
            robots_inds_2 = robots_inds[:]
            while (len(robots_inds_2) > 0):
                p = robots_inds_2.pop(0)
                robots_inds_f[p] = [p]
                if len(robots_inds_2) == 0:
                    break
                for ind_j in robots_inds_2:
                    # if not (ard.reached[p] and ard.reached[ind_j]):
                    dist = cal_distance(ard.x[p], ard.y[p], ard.x[ind_j], ard.y[ind_j])
                    if (dist < (self.p.robot_prec_d*2.2)):    # param 2
                        robots_inds_f[p].append(ind_j)

            # detect groups
            robots_inds_3 = robots_inds[:]
            while len(robots_inds_3) > 0:
                groups.append([])
                p = robots_inds_3.pop(0)
                gset = set(robots_inds_f[p])
                robots_inds_4 = robots_inds_3[:]
                for ind_j in robots_inds_4:
                    nset = set(robots_inds_f[ind_j])
                    if len(gset.intersection(nset)) > 0:
                        gset = gset.union(nset)
                        robots_inds_3.remove(ind_j)
                groups[-1] = list(gset)

        # groups - new_robots -----------------------------------
        for g in groups:
            if len(g) > 1:
                self.is_multi = True
                nr = ApfRobot()
                nr.big = True
                is_robot_in = False
                is_target_in = False

                # priorities
                P = [ard.pr[i] > 0 for i in g]

                # polygon
                is_g2 = True
                if len(g) > 2:
                    point_robot = Point(self.pose.x, self.pose.y)
                    point_target = Point(self.goal_x, self.goal_y)
                    polys_points = [Point(ard.x[i], ard.y[i])
                                    for i in g if (not ard.reached[i])]
                    if (len(polys_points) > 2):
                        is_g2 = False
                        mpt = MultiPoint([shape(p) for p in polys_points])
                        mp = mpt.convex_hull
                        mp_bound = mp.boundary.coords
                        mpc = mp.centroid.coords[0]
                        is_robot_in = mp.contains(point_robot)
                        is_target_in = mp.contains(point_target)
                        self.mp_bound = [[], mp_bound]

                        # get the minimum bounding circle of the convex hull
                        mbr = mp.minimum_rotated_rectangle
                        circum_center = mbr.centroid.coords[0]
                        # calculate the radius of the circumscribed circle
                        radius = mbr.exterior.distance(mbr.centroid)
                        xc = circum_center[0]
                        yc = circum_center[1]
                        rc = c_radius*radius  # + self.p.robot_r + self.p.prec_d    # param 3
                        rc = max(rc, 2*self.p.robot_prec_d)

                # if robot is in the polygon
                if (not is_target_in) and is_robot_in:
                    self.stop_flag_multi = True
                    return

                # robot is not in the polygon, detect """triangle"""
                if is_g2:
                    ad = [AD_h_rR[i] for i in g]
                    a_min = g[np.argmin(ad)]
                    a_max = g[np.argmax(ad)]

                    x1 = ard.x[a_min]
                    x2 = ard.x[a_max]
                    y1 = ard.y[a_min]
                    y2 = ard.y[a_max]
                    dx = x2-x1
                    dy = y2-y1
                    theta = np.arctan2(dy, dx)
                    d12 = cal_distance(x1, y1, x2, y2)

                    xx1 = x1 + (d12/np.sqrt(3)) * np.cos(theta+np.pi/6)
                    yy1 = y1 + (d12/np.sqrt(3)) * np.sin(theta+np.pi/6)
                    xx2 = x1 + (d12/np.sqrt(3)) * np.cos(theta-np.pi/6)
                    yy2 = y1 + (d12/np.sqrt(3)) * np.sin(theta-np.pi/6)
                    dd1 = cal_distance(xx1, yy1, self.pose.x, self.pose.y)
                    dd2 = cal_distance(xx2, yy2, self.pose.x, self.pose.y)
                    if (dd1 < dd2):
                        xc = xx2
                        yc = yy2
                    else:
                        xc = xx1
                        yc = yy1
                    rc = d12/np.sqrt(2)      # /np.sqrt(3) d12 #$

                #
                d_tc = cal_distance(self.goal_x, self.goal_y, xc, yc)
                if (d_tc < rc):
                    is_target_in = True
                    continue

                # r_c, r_R
                dx = xc - self.pose.x
                dy = yc - self.pose.y
                d_rR = np.sqrt(dx**2 + dy**2)
                theta_rR = np.arctan2(dy, dx)
                ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)

                nr.x = xc
                nr.y = yc
                nr.d = d_rR
                nr.h_rR = ad_h_rR
                nr.theta_rR = theta_rR
                if any(P):
                    nr.p = True
                nr.r_prec = rc + self.p.robot_r + self.p.prec_d
                # nr.r_prec = self.eval_obst(xc, yc, nr.r_prec, d_rR)
                nr.r_half = 1.5 * nr.r_prec
                nr.r_start = 2.0 * nr.r_prec
                nr.z = 4 * self.p.fix_f * nr.r_prec**4
                # new_robots.append(nr)
                multi_robots.append(nr)

        if len(multi_robots) > 1:
            rcs = [mr.r_prec for mr in multi_robots]
            rcs = np.array(rcs)
            max_ind = np.argmax(rcs)
            multi_robots = [multi_robots[max_ind]]
            multi_robots_vis.append(multi_robots[0])
            new_robots.append(multi_robots[0])
        elif len(multi_robots) == 1:
            multi_robots_vis.append(multi_robots[0])
            new_robots.append(multi_robots[0])

        #
        self.multi_robots_vis = multi_robots_vis
        self.new_robots = new_robots
        return

    def detect_obsts(self):
        f_obsts_inds = []
        for oi in self.obs_ind_main:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            do = cal_distance(xo, yo, self.pose.x, self.pose.y)
            if do < self.p.obst_start_d:
                f_obsts_inds.append(oi)
        self.f_obsts_inds = f_obsts_inds

    def eval_obst(self, xc, yc, rc, d_rR):
        xy = []
        # ros = [rc]
        for oi in self.obs_ind_main:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            d_Ro = cal_distance(xo, yo, xc, yc)
            d_ro = cal_distance(xo, yo, self.pose.x, self.pose.y)
            if True:  # d_rR>rc: #d_ro<d_rR and
                if (rc > (d_Ro-self.p.obst_prec_d)) and rc < (d_Ro+self.p.obst_prec_d):
                    # ros.append(d_Ro+self.p.obst_prec_d*1)
                    x = (xo+xc)/2
                    y = (yo+yc)/2
                    xy.append([x, y])

        # if ros!=[]:
        #     do_max = max(ros)
        #     rc = do_max
        return xy  # rc

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        # f = self.p.zeta * goal_dist
        f = self.p.fix_f
        if goal_dist < 0.5:
            f = 2*f
        theta_rg = np.arctan2(dy, dx)
        ad_rg_h = cal_angle_diff(theta_rg, self.pose.theta)
        self.ad_rg_h = ad_rg_h
        self.theta_rg = theta_rg
        self.goal_dist = goal_dist
        self.goal_theta = theta_rg
        fx = round(f * np.cos(ad_rg_h), 3)
        fy = round(f * np.sin(ad_rg_h), 3)
        self.target_f = [fx, fy]

    def f_robots(self):

        robot_f = [0, 0]
        self.robot_f = [0, 0]
        new_robots = self.new_robots

        for nr in new_robots:
            nr_force = [0, 0]
            if (not nr.big):
                nr_force = self.compute_robot_force(nr)
            else:
                # if (not self.near_robots) and (not self.near_obst):
                nr_force = self.compute_multi_force(nr)

            robot_f[0] += round(nr_force[0], 3)
            robot_f[1] += round(nr_force[1], 3)

        coeff_f = 1
        self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
        self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    def compute_multi_force(self, nr):
        nr_force = [0, 0]

        # tocheck #todo
        if (nr.r_start < nr.d):
            return nr_force

        # force
        coeff = 1
        f1 = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2
        f = f1 + 2
        nr_force = [f * -np.cos(nr.h_rR), f * np.sin(nr.h_rR)]

        # based_on_goal, target_other_side
        based_on_goal = False
        dx = self.goal_x - nr.x
        dy = self.goal_y - nr.y
        theta_Rg = np.arctan2(dy, dx)
        theta_Rr = nr.theta_rR - np.pi
        ad_Rg_Rr = cal_angle_diff(theta_Rg, theta_Rr)
        if abs(ad_Rg_Rr) < np.deg2rad(180-20):
            based_on_goal = True
        target_other_side = False
        if abs(ad_Rg_Rr) > np.pi/5:
            target_other_side = True

        theta_ = 20
        ad_rg_rR = cal_angle_diff(self.theta_rg,  nr.theta_rR)
        if based_on_goal:
            coeff = np.sign(ad_rg_rR*nr.h_rR)
        else:
            if abs(nr.h_rR) < np.deg2rad(theta_):
                coeff = np.sign(ad_rg_rR*nr.h_rR)

        angle_turn_r = nr.theta_rR + (np.pi/2+np.pi/8)*np.sign(nr.h_rR)*coeff
        ad_c_h = cal_angle_diff(angle_turn_r, self.pose.theta)
        f3 = f1 + 4
        templ3 = [f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h)]

        if target_other_side:
            if (nr.r_prec < nr.d):
                # if (nr.r_prec<nr.d) and abs(nr.h_rR)<np.pi/2: # todo
                nr_force = templ3
            elif (0.8*nr.r_prec < nr.d < nr.r_prec):
                nr_force = templ3
                # nr_force = [templ3[0]+nr_force[0], templ3[1]+nr_force[1]]

        return nr_force

    def compute_robot_force(self, nr):
        nr_force = [0, 0]
        if (nr.d < nr.r_start):

            # near_robots
            if (nr.d < nr.r_half):
                self.near_robots = True

            #
            templ2 = []
            templ3 = []
            templ3_2 = []
            nr_force = []

            # target_other_side
            dx = self.goal_x - nr.x
            dy = self.goal_y - nr.y
            d_Rg = cal_distance(self.goal_x, self.goal_y, nr.x, nr.y)
            theta_Rg = np.arctan2(dy, dx)
            theta_Rr = nr.theta_rR - np.pi
            ad_Rg_Rr = cal_angle_diff(theta_Rg, theta_Rr)
            target_other_side = False
            if abs(ad_Rg_Rr) > np.pi/3:
                target_other_side = True

            # r_coeff, angle_turn_r
            r_coeff = 1
            theta_ = 10
            ad_h_rR = nr.h_rR
            if abs(ad_h_rR) < np.deg2rad(theta_):
                ad_rg_rR = cal_angle_diff(self.theta_rg,  nr.theta_rR)
                r_coeff = np.sign(ad_rg_rR*nr.h_rR)

            # R_coeff, angle_turn_R
            R_coeff = 1
            flag_rR = True
            ad_Rr_H = cal_angle_diff((nr.theta_rR - np.pi), nr.H)
            ad_rR_h = cal_angle_diff(nr.theta_rR, self.pose.theta)
            if (ad_Rr_H*ad_rR_h) < 0:
                if nr.p:
                    self.stop_flag_robots
                    R_coeff = -1
                #     print(self.rid, " ==== ")
                # if abs(ad_rR_h)>abs(ad_Rr_H):
                #     R_coeff = -1
                #     flag_rR = False

            # stops
            if (nr.d < nr.r_prec):
                if (abs(nr.h_rR) < (np.pi/4)):  # to check # np.pi/4 np.pi/2
                    self.stop_flag_robots = True
                if (not nr.reached) and (not nr.stop):  # and nr.p:
                    if abs(ad_rR_h) < np.pi/2 and abs(ad_Rr_H) > np.pi/2:
                        self.stop_flag_full = True
                        # return [0, 0]

            # case hard!
            ad = cal_angle_diff(theta_Rg,  self.theta_rg)
            if (nr.p) and abs(ad) < np.deg2rad(30) and d_Rg < self.goal_dist and abs(self.ad_rg_h) < np.deg2rad(40):
                if abs(cal_angle_diff(self.pose.theta, nr.H)) < np.deg2rad(90):
                    self.stop_flag_full = True

            # angle_turns
            angle_turn_R = nr.theta_rR - (np.pi/2)*np.sign(ad_Rr_H*R_coeff)
            ad_C_h = cal_angle_diff(angle_turn_R, self.pose.theta)
            angle_turn_r = nr.theta_rR + (np.pi/2)*np.sign(ad_h_rR)*r_coeff
            ad_c_h = cal_angle_diff(angle_turn_r, self.pose.theta)

            # forces
            f = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2
            fl = f + 0
            nr_force = [fl * -np.cos(ad_h_rR), fl * np.sin(ad_h_rR)]

            f2 = f + 2
            f2_2 = f + 4
            templ2 = [f2 * np.cos(ad_C_h), f2 * np.sin(ad_C_h)]
            templ2_2 = [f2_2 * np.cos(ad_C_h), f2_2 * np.sin(ad_C_h)]

            f3 = f + 2  # tocheck
            f3_2 = f + 4
            templ3 = [f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h)]
            templ3_2 = [f3_2 * np.cos(ad_c_h), f3_2 * np.sin(ad_c_h)]

            # adjust heading
            if target_other_side:
                if (nr.r_half < nr.d < nr.r_start):
                    if (not nr.reached) and (not nr.stop):
                        if (flag_rR and abs(ad_h_rR) < np.pi/2) and (abs(ad_Rr_H) < (np.pi/2)):
                            nr_force = [templ2[0]+nr_force[0],
                                        templ2[1]+nr_force[1]]
                    else:
                        if (abs(ad_h_rR) < (np.pi/2)):
                            nr_force = [templ3[0]+nr_force[0],
                                        templ3[1]+nr_force[1]]

                elif (nr.r_prec < nr.d < nr.r_half):
                    if (not nr.reached) and (not nr.stop):
                        if (flag_rR and abs(ad_h_rR) < np.pi/2 and abs(ad_Rr_H) < np.pi/2):
                            nr_force = [templ2_2[0]+nr_force[0],
                                        templ2_2[1]+nr_force[1]]
                    else:
                        if True:  # (abs(ad_h_rR)<(np.pi/2)):
                            nr_force = [templ3_2[0]+nr_force[0],
                                        templ3_2[1]+nr_force[1]]
        return nr_force

    def f_obstacle(self):
        obs_f = [0, 0]
        self.obs_f = [0, 0]

        for i in self.f_obsts_inds:
            dy = (self.obs_y[i] - self.pose.y)
            dx = (self.obs_x[i] - self.pose.x)
            d_ro = np.sqrt(dx**2 + dy**2)

            theta_ro = np.arctan2(dy, dx)
            ad_h_ro = cal_angle_diff(self.pose.theta, theta_ro)

            if (d_ro < self.p.obst_half_d):
                self.near_obst = True

            # if (d_ro < self.p.obst_prec_d) and (abs(ad_h_ro)<(np.pi/2)):
            #     self.stop_flag_obsts = True

            dx = self.goal_x - self.obs_x[i]
            dy = self.goal_y - self.obs_y[i]
            theta_og = np.arctan2(dy, dx)
            theta_or = theta_ro - np.pi
            ad_og_or = cal_angle_diff(theta_og, theta_or)
            target_other_side = False
            if abs(ad_og_or) > np.pi/4:
                target_other_side = True

            coeff = 1
            theta_ = 20
            if abs(ad_h_ro) < np.deg2rad(theta_):
                ad_rg_ro = cal_angle_diff(self.theta_rg,  theta_ro)
                coeff = np.sign(ad_rg_ro*ad_h_ro)
            # angle_turn_o = theta_ro + (np.pi/2)*np.sign(ad_h_ro)
            # ad_c_o = cal_angle_diff(angle_turn_o, self.pose.theta)
            angle_turn_t = theta_ro + (np.pi/2)*np.sign(ad_h_ro)*coeff
            ad_c_t = cal_angle_diff(angle_turn_t, self.pose.theta)

            f = ((self.p.obst_z * 1) * ((1 / d_ro) -
                 (1 / self.p.obst_start_d))**2) * (1 / d_ro)**2
            o_force = [f * -np.cos(ad_h_ro), f * np.sin(ad_h_ro)]

            # fo = f + 2
            # templo = [fo * np.cos(ad_c_o), fo * np.sin(ad_c_o)]

            ft = f + 3
            templt = [ft * np.cos(ad_c_t), ft * np.sin(ad_c_t)]

            if target_other_side:
                if (self.p.obst_prec_d < d_ro and d_ro < self.p.obst_half_d):
                    o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]
                elif (self.p.obst_half_d < d_ro):
                    if (abs(ad_h_ro) < np.pi/2):
                        o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]

            obs_f[0] += round(o_force[0], 3)
            obs_f[1] += round(o_force[1], 3)

        coeff_f = 1
        self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
        self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    def map_data(self):
        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y
        self.obs_count = self.model.obst.count
        self.obs_ind_main = [i for i in range(self.model.obst.count)]

    def cal_vel(self):

        #
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.p.v_max * ((f_r / self.p.fix_f)**2)  # + self.p.v_min_2
        # w
        w = 5 * self.p.w_max * f_theta / self.p.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v == 0) and abs(w) < 0.03:
            v = self.p.v_min_2*1

        # thresh_theta = np.pi/3
        # w = 4 * self.p.w_max * theta / (np.pi/6)
        # v = 3 * self.p.v_max * (1-abs(theta)/thresh_theta)

        # if (v<self.p.v_min_2) and abs(w)<0.03:
        #     v = self.p.v_min_2*2

        # check bounds
        v = min(v, self.p.v_max)
        v = max(v, self.p.v_min)
        wa = min(abs(w), self.p.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w
