#! /usr/bin/env python

from typing import List, Dict, Tuple
from array import array
import numpy as np
from shapely.coords import CoordinateSequence  # type: ignore # pylint: disable-all
from shapely.geometry import Point, shape, MultiPoint  # type: ignore # pylint: disable-all
# from shapely.geometry.polygon import Polygon
from logger import MyLogger
from parameters import Params
from mrapf_classes import ApfRobot
from my_utils import cal_distance
from my_utils import cal_angle_diff
from create_model import MRSModel, Robot
from apf.msg import RobotData, FleetData
from apf_planner_base import APFPlannerBase


class APFPlanner(APFPlannerBase):

    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        APFPlannerBase.__init__(self, model, robot, params)
        self.lg = MyLogger(f"APFPlanner2_r{robot.rid}")

        #
        self.apf_robots: List[ApfRobot] = []
        self.multi_robots_vis: List[ApfRobot] = []
        self.cluster_poly_xy: List[Tuple[array, array]] = []

        #
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False

        # config
        self.c_radius = 2.5  # 2.5          #$ param 3
        self.c_r = 2.5     # 2.5 3.0      #$ param 1

    def reset_vals_2(self):
        #
        self.multi_robots_vis = []
        self.cluster_poly_xy = []
        #
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
        self.stop_flag_multi = False
        #
        self.apf_robots = []

    def calculate_planner_forces(self) -> bool:
        """ planner specific force calculations

        Returns:
            bool: True if robot can move, False if stopped
        """

        self.reset_vals_2()

        # detect obstacles
        self.detect_obstacles_in_proximity()

        # detect and group
        self.create_all_fake_obstacles()

        # check stop_flag_multi
        if self.stop_flag_multi:
            self.lg.warn("stop_flag_multi activated.")
            self.stopped = True
            self.v = 0
            return False
        else:
            # calculate forces
            self.calculate_forces()

            # check stop flags
            if self.stop_flag_obsts or self.stop_flag_robots:
                self.lg.warn("stop_flag_obsts or stop_flag_robots activated.")
                self.stopped = True
                self.v = 0
                if abs(self.w) < (np.deg2rad(2)):
                    self.v = self.params.v_min_2
                return False

            # check stop_flag_full
            if self.stop_flag_full:
                self.lg.warn("stop_flag_full activated.")
                self.stopped = True
                self.v = 0
                return False
        return True

    def create_all_fake_obstacles(self) -> None:
        """ create all fake obstacles from fleet data. \n
        includes local minima avoidance and clustering.
        """

        # reset
        is_goal_close = False
        apf_robots: List[ApfRobot] = []
        AD_h_rR: Dict[int, float] = {}
        cluster_r_inds_1: List[int] = []
        multi_robots: List[ApfRobot] = []
        multi_robots_vis: List[ApfRobot] = []

        # is_goal_close
        goal_dist = cal_distance(self.pose.x, self.pose.y, self.goal_x, self.goal_y)
        if (goal_dist < (self.c_r*self.params.robot_start_d)):
            is_goal_close = True

        # evaluate fleet data ##################################################
        # create new apf_robots and
        apf_robots, cluster_r_inds_1, AD_h_rR = self.eval_fleet_data()

        # create fake_apf_robots_1 for local minima avoidance ##################
        # fake_apf_robots_1: List[ApfRobot]
        fake_apf_robots_1 = self.create_fake_obsts_loc_min_avoid(apf_robots)

        # add fake_apf_robots_1 to the apf_robots and multi_robots_vis lists
        apf_robots.extend(fake_apf_robots_1)
        multi_robots_vis.extend(fake_apf_robots_1)

        # if there is none robots in proximity
        # if goal is close, don't create fake obstacles from clustering
        if not (is_goal_close or len(cluster_r_inds_1) == 0):

            # create fake obstacles from clustering ############################
            multi_robots = self.create_fake_obst_clusters(cluster_r_inds_1, AD_h_rR)
            if len(multi_robots) > 0:
                multi_robots_vis.append(multi_robots[0])
                apf_robots.append(multi_robots[0])

        #
        self.multi_robots_vis = multi_robots_vis
        self.apf_robots = apf_robots
        return

    def detect_obstacles_in_proximity(self):
        """ detect obstacles in proximity circle"""
        f_obsts_inds = []
        for oi in self.obst_orig_inds:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            do = cal_distance(xo, yo, self.pose.x, self.pose.y)
            if do < self.params.obst_start_d:
                f_obsts_inds.append(oi)
        self.f_obsts_inds = f_obsts_inds

    def eval_fleet_data(self) -> Tuple[List[ApfRobot], List[int], Dict[int, float]]:
        """
        evaluate fleet data

        Returns:
            tuple containing
            - apf_robots(List[ApfRobot]): list of new ApfRobot instances based on fleet data
            - cluster_r_inds_1(List[int]): list of robots indices for clustering
            - AD_h_rR(Dict[int, float]): angle differences of heading with rR to other robots
        """

        #
        apf_robots: List[ApfRobot] = []
        cluster_r_inds_1: List[int] = []
        AD_h_rR: Dict[int, float] = {}

        # eval fleet data
        # find robots in proximity circle for clustering and create ApfRobot instances
        fd: FleetData = self.fleet_data
        orob: RobotData = None
        for orob in fd.fdata:
            if orob.rid == self.robot.rid:
                continue
            dx = (orob.x - self.pose.x)
            dy = (orob.y - self.pose.y)
            d_rR = np.sqrt(dx**2 + dy**2)
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = cal_angle_diff(self.pose.theta, theta_rR)
            AD_h_rR[orob.rid] = ad_h_rR
            # ad_h_rR_abs = abs(ad_h_rR)
            # ad_H_Rr = cal_angle_diff(orob.h, (theta_rR - np.pi))
            # ad_H_Rr_abs = abs(ad_H_Rr)

            # check distance for clustering
            if (d_rR > (self.c_r * self.params.robot_start_d)):
                continue

            # if (not orob.reached) or (d_rR < (1 * self.params.robot_start_d)):
            # if (d_rR < (1 * self.params.robot_start_d)) or ((not orob.reached) or (ad_h_rR_abs < np.pi/2 or ad_H_Rr_abs < np.pi/2)):

            if (not orob.reached) or (d_rR < (1 * self.params.robot_start_d)):
                cluster_r_inds_1.append(orob.rid)

            # create ApfRobot instances as individual robots
            if d_rR < (1 * self.params.robot_start_d):
                new_robot = self.create_new_apf_robot(orob, d_rR, ad_h_rR, theta_rR)
                apf_robots.append(new_robot)

        #
        return apf_robots, cluster_r_inds_1, AD_h_rR

    def create_fake_obsts_loc_min_avoid(self, apf_robots: List[ApfRobot]) -> List[ApfRobot]:
        """ create fake obstacles between robot and obstacle.
            for local minima avoidance
        """
        fake_apf_robots_1: List[ApfRobot] = []
        for orob in apf_robots:
            if not orob.reached:
                continue
            XY: List[Tuple[float, float]] = self.eval_for_fake_obst_min_loc_avoid(orob.x, orob.y, orob.r_prec, orob.d)
            for xy in XY:
                dx_ = (xy[0] - self.pose.x)
                dy_ = (xy[1] - self.pose.y)
                d_rR_ = np.sqrt(dx_**2 + dy_**2)
                theta_rR_ = np.arctan2(dy_, dx_)
                ad_h_rR_ = cal_angle_diff(self.pose.theta, theta_rR_)
                robo = ApfRobot(x=xy[0], y=xy[1], d=d_rR_, H=orob.H)
                robo.h_rR = ad_h_rR_
                robo.theta_rR = theta_rR_
                robo.prior = orob.priority > self.robot_data.priority
                robo.stopped = True
                robo.reached = True
                robo.r_prec = orob.r_prec/1.5  # $
                robo.r_half = 1.5 * robo.r_prec
                robo.r_start = 2 * robo.r_prec
                robo.z = 4 * self.params.fix_f * robo.r_prec**4
                fake_apf_robots_1.append(robo)
        return fake_apf_robots_1

    def eval_for_fake_obst_min_loc_avoid(self, xc, yc, prec_d, d_rR) -> List[Tuple[float, float]]:
        """ evaluate for fake obstacles between robot and obstacle.
            for local minima avoidance
        """
        xy: List[Tuple[float, float]] = []
        for oi in self.obst_orig_inds:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            d_Ro = cal_distance(xo, yo, xc, yc)
            # d_ro = cal_distance(xo, yo, self.pose.x, self.pose.y)
            if True:  # d_rR>rc: #d_ro<d_rR and
                if (prec_d > (d_Ro-self.params.obst_prec_d)) and prec_d < (d_Ro+self.params.obst_prec_d):
                    # ros.append(d_Ro+self.params.obst_prec_d*1)
                    x = (xo+xc)/2
                    y = (yo+yc)/2
                    xy.append((x, y))

        # if ros!=[]:
        #     do_max = max(ros)
        #     rc = do_max
        return xy  # rc

    def create_fake_obst_clusters(self, cluster_r_inds_1: List[int], AD_h_rR: Dict[int, float]) -> List[ApfRobot]:
        """ create fake obstacles from clustering
        """

        #
        robots_inds_f = {}
        groups = []
        multi_robots: List[ApfRobot] = []

        # fleet data
        fd: FleetData = self.fleet_data

        # generate robots_inds_f (neighbor robots in proximity circle)
        robots_inds_2 = cluster_r_inds_1[:]
        while (len(robots_inds_2) > 0):
            p = robots_inds_2.pop(0)
            robots_inds_f[p] = [p]
            if len(robots_inds_2) == 0:
                break
            for ind_j in robots_inds_2:
                # if not (fd.fdata[p].reached and fd.fdata[ind_j].reached):
                dist = cal_distance(fd.fdata[p].x, fd.fdata[p].y, fd.fdata[ind_j].x, fd.fdata[ind_j].y)
                if (dist < (self.params.robot_prec_d*2.2)):    # param 2
                    robots_inds_f[p].append(ind_j)

        # detect groups
        robots_inds_3 = cluster_r_inds_1[:]
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

        # groups - apf_robots -----------------------------------
        for g in groups:
            if len(g) > 1:
                nr = ApfRobot()
                nr.cluster = True
                is_robot_in = False
                is_target_in = False

                # priorities
                P = [fd.fdata[i].priority > self.robot_data.priority for i in g]

                # polygon
                is_g2 = True
                if len(g) > 2:
                    point_robot = Point(self.pose.x, self.pose.y)
                    point_target = Point(self.goal_x, self.goal_y)
                    polys_points = [Point(fd.fdata[i].x, fd.fdata[i].y) for i in g if (not fd.fdata[i].reached)]
                    if (len(polys_points) > 2):
                        is_g2 = False
                        mpt = MultiPoint([shape(p) for p in polys_points])
                        mp = mpt.convex_hull
                        cluster_poly_coords: CoordinateSequence = mp.boundary.coords
                        # mpc = mp.centroid.coords[0]
                        is_robot_in = mp.contains(point_robot)
                        is_target_in = mp.contains(point_target)
                        self.cluster_poly_xy.append(cluster_poly_coords.xy)

                        # get the minimum bounding circle of the convex hull
                        mbr = mp.minimum_rotated_rectangle
                        circum_center = mbr.centroid.coords[0]
                        # calculate the radius of the circumscribed circle
                        radius = mbr.exterior.distance(mbr.centroid)
                        xc = circum_center[0]
                        yc = circum_center[1]
                        rc = self.c_radius*radius  # + self.params.robot_r + self.params.prec_d    # param 3
                        rc = max(rc, 2*self.params.robot_prec_d)

                # if robot is in the polygon
                if (not is_target_in) and is_robot_in:
                    self.stop_flag_multi = True
                    return

                # robot is not in the polygon, detect """triangle"""
                if is_g2:
                    ad = [AD_h_rR[i] for i in g]
                    a_min = g[np.argmin(ad)]
                    a_max = g[np.argmax(ad)]

                    x1 = fd.fdata[a_min].x
                    x2 = fd.fdata[a_max].x
                    y1 = fd.fdata[a_min].y
                    y2 = fd.fdata[a_max].y
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
                    nr.prior = True
                nr.r_prec = rc + self.params.robot_r + self.params.prec_d
                # nr.r_prec = self.eval_obst(xc, yc, nr.r_prec, d_rR)
                nr.r_half = 1.5 * nr.r_prec
                nr.r_start = 2.0 * nr.r_prec
                nr.z = 4 * self.params.fix_f * nr.r_prec**4
                # apf_robots.append(nr)
                multi_robots.append(nr)

        if len(multi_robots) > 1:
            rcs = [mr.r_prec for mr in multi_robots]
            rcs = np.array(rcs)
            max_ind = np.argmax(rcs)
            multi_robots = [multi_robots[max_ind]]
        return multi_robots

    def create_new_apf_robot(self, orob: RobotData, d_rR: float, ad_h_rR: float, theta_rR: float) -> ApfRobot:
        """ create new ApfRobot instance for calculating forces
        """
        robo = ApfRobot(x=orob.x, y=orob.y, d=d_rR, H=orob.h)
        robo.h_rR = ad_h_rR
        robo.theta_rR = theta_rR
        robo.priority = orob.priority
        robo.prior = (not (orob.reached or orob.stopped)) and (orob.priority > self.robot_data.priority)
        robo.stopped = orob.stopped
        robo.reached = orob.reached
        rc = self.params.robot_prec_d
        robo.r_prec = rc
        robo.r_half = 1.5 * rc
        robo.r_start = 2.0 * rc
        robo.z = 4 * self.params.fix_f * rc**4
        return robo

    def f_target(self):
        # r_g
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        goal_dist = np.sqrt(dx**2 + dy**2)
        # f = self.params.zeta * goal_dist
        f = self.params.fix_f
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
        self.target_f = (fx, fy)

    def f_robots(self):
        fx, fy = 0.0, 0.0
        new_robots = self.apf_robots
        for nr in new_robots:
            nr_force: Tuple[float, float] = (0, 0)
            if (not nr.cluster):
                nr_force = self.compute_robot_force(nr)
            else:
                # if (not self.near_robots) and (not self.near_obst):
                nr_force = self.compute_multi_force(nr)

            fx += nr_force[0]
            fy += nr_force[1]

        coeff_f = 1
        fx = round(fx * coeff_f, 3)
        fx = round(fy * coeff_f, 3)
        self.robot_f = (fx, fy)

    def compute_multi_force(self, nr: ApfRobot) -> Tuple[float, float]:
        nr_force = (0, 0)

        # tocheck #todo
        if (nr.r_start < nr.d):
            return nr_force

        # force
        coeff = 1
        f1 = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2
        f = f1 + 2
        nr_force = (f * -np.cos(nr.h_rR), f * np.sin(nr.h_rR))

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
        templ3 = (f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h))

        if target_other_side:
            if (nr.r_prec < nr.d):
                # if (nr.r_prec<nr.d) and abs(nr.h_rR)<np.pi/2: # todo
                nr_force = templ3
            elif (0.8*nr.r_prec < nr.d < nr.r_prec):
                nr_force = templ3
                # nr_force = [templ3[0]+nr_force[0], templ3[1]+nr_force[1]]

        return nr_force

    def compute_robot_force(self, nr: ApfRobot) -> Tuple[float, float]:
        if (nr.d > nr.r_start):
            return (0.0, 0.0)

        # near_robots
        if (nr.d < nr.r_half):
            self.near_robots = True

        #
        templ2 = []
        templ3 = []
        templ3_2 = []

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
            if nr.prior:
                self.stop_flag_robots
                R_coeff = -1
            # if abs(ad_rR_h)>abs(ad_Rr_H):
            #     R_coeff = -1
            #     flag_rR = False

        # stops
        if (nr.d < nr.r_prec):
            if (abs(nr.h_rR) < (np.pi/4)):  # to check # np.pi/4 np.pi/2
                self.stop_flag_robots = True
            if (not nr.reached) and (not nr.stopped):  # and nr.prior:
                if abs(ad_rR_h) < np.pi/2 and abs(ad_Rr_H) > np.pi/2:
                    self.stop_flag_full = True
                    # return [0, 0]

        # case hard!
        ad = cal_angle_diff(theta_Rg,  self.theta_rg)
        if (nr.prior) and abs(ad) < np.deg2rad(30) and d_Rg < self.goal_dist and abs(self.ad_rg_h) < np.deg2rad(40):
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
        nr_force = (fl * -np.cos(ad_h_rR), fl * np.sin(ad_h_rR))

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
                if (not nr.reached) and (not nr.stopped):
                    if (flag_rR and abs(ad_h_rR) < np.pi/2) and (abs(ad_Rr_H) < (np.pi/2)):
                        nr_force = (templ2[0]+nr_force[0], templ2[1]+nr_force[1])
                else:
                    if (abs(ad_h_rR) < (np.pi/2)):
                        nr_force = (templ3[0]+nr_force[0], templ3[1]+nr_force[1])

            elif (nr.r_prec < nr.d < nr.r_half):
                if (not nr.reached) and (not nr.stopped):
                    if (flag_rR and abs(ad_h_rR) < np.pi/2 and abs(ad_Rr_H) < np.pi/2):
                        nr_force = (templ2_2[0]+nr_force[0], templ2_2[1]+nr_force[1])
                else:
                    if True:  # (abs(ad_h_rR)<(np.pi/2)):
                        nr_force = (templ3_2[0]+nr_force[0], templ3_2[1]+nr_force[1])
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

            if (d_ro < self.params.obst_half_d):
                self.near_obst = True

            # if (d_ro < self.params.obst_prec_d) and (abs(ad_h_ro)<(np.pi/2)):
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

            f = ((self.params.obst_z * 1) * ((1 / d_ro) - (1 / self.params.obst_start_d))**2) * (1 / d_ro)**2
            o_force = [f * -np.cos(ad_h_ro), f * np.sin(ad_h_ro)]

            # fo = f + 2
            # templo = [fo * np.cos(ad_c_o), fo * np.sin(ad_c_o)]

            ft = f + 3
            templt = [ft * np.cos(ad_c_t), ft * np.sin(ad_c_t)]

            if target_other_side:
                if (self.params.obst_prec_d < d_ro and d_ro < self.params.obst_half_d):
                    o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]
                elif (self.params.obst_half_d < d_ro):
                    if (abs(ad_h_ro) < np.pi/2):
                        o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]

            obs_f[0] += round(o_force[0], 3)
            obs_f[1] += round(o_force[1], 3)

        coeff_f = 1
        self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
        self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    def calculate_velocity(self):
        #
        f_r, f_theta = self.f_r, self.f_theta

        # v
        if f_r < 0:
            v = 0
        else:
            v = 1 * self.params.v_max * ((f_r / self.params.fix_f)**2)  # + self.params.v_min_2
        # w
        w = 5 * self.params.w_max * f_theta / self.params.fix_f
        if f_r < -1 and abs(w) < 0.05:
            w = 1*np.sign(w)

        # v
        if (v == 0) and abs(w) < 0.03:
            v = self.params.v_min_2*1

        # check bounds
        v = min(v, self.params.v_max)
        v = max(v, self.params.v_min)
        wa = min(abs(w), self.params.w_max)
        w = wa * np.sign(w)
        self.v = v
        self.w = w
