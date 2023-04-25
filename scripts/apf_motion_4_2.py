#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization import Viusalize
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion

# from shapely.geometry.polygon import Polygon
from shapely.geometry import Point, shape, MultiPoint


class NewRobots:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 1
        self.d = 0
        self.H = 0
        self.h_rR = 0
        self.theta_rR = 0
        self.r_prec = 0
        self.r_half = 0
        self.r_start = 0
        self.p = False
        self.big = False
        self.stop = False
        self.reached = False


class ApfMotion(object):

    def __init__(self, model, robot, init_params):

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # Viusalize
        self.vs = Viusalize(model)

        # preallocation - params - setting
        self.init(model, robot, init_params)

        # map: target and obstacles coordinates
        self.map()

        # /cmd_vel puplisher
        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)

        # listener
        self.check_topic()
        rospy.Subscriber(self.topic, self.topic_type, self.get_odom)

        # pose service client
        rospy.wait_for_service(self.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(self.pose_srv_name, SharePoses2)

        # execute goal
        self.exec_cb()

        # # tensor force
        # self.tensor_force()

    # --------------------------  init  ---------------------------#

    def init(self, model, robot, init_params):
        # preallocation
        self.r_x = 0
        self.r_y = 0
        self.r_h = 0
        self.theta_rg = 0
        self.goal_theta = 0
        self.goal_dist = 1000
        self.phis = []
        self.v_lin = []
        self.v_ang = []
        self.path_x = []
        self.path_y = []
        self.force_tr = []
        self.force_tt = []
        self.force_or = []
        self.force_ot = []
        self.is_multi = False
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False

        # data
        self.model = model
        self.robot = robot

        # parameters
        self.topic_type = Odometry
        self.prioriy = robot.priority

        # params
        self.ind = init_params.id
        self.ns = init_params.name_space
        self.topic = init_params.lis_topic
        self.cmd_topic = init_params.cmd_topic
        self.pose_srv_name = init_params.pose_srv_name

        # parameters vel
        self.v = 0
        self.w = 0
        self.v_max = 0.2        # init_params.linear_max_speed
        self.v_min = 0.0        # init_params.linear_min_speed
        self.w_min = 0.0        # init_params.angular_min_speed
        self.w_max = 1.0        # init_params.angular_max_speed
        self.v_min_2 = 0.05     # init_params.linear_min_speed_2

        # settings
        self.zeta = 1
        self.fix_f = 4
        self.fix_f2 = 10
        self.obst_r = 0.11
        self.prec_d = 0.07
        self.robot_r = 0.22

        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = 2 * self.obst_prec_d
        self.obst_half_d = 1.5 * self.obst_prec_d
        self.obst_z = 4 * self.fix_f * self.obst_prec_d**4

        self.robot_prec_d = 2 * self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = 2 * self.robot_prec_d
        self.robot_half_d = 1.5 * self.robot_prec_d
        self.robot_z = 4 * self.fix_f * self.robot_prec_d**4

        self.w_coeff = 1                        # init_params.w_coeff       # angular velocity coeff
        self.goal_dis_tresh = 0.06              # init_params.dis_tresh     # distance thresh to finish
        self.theta_thresh = np.deg2rad(30)      # init_params.theta_thresh  # for velocity calculation

    # --------------------------  exec_cb  ---------------------------#

    def exec_cb(self):
        self.go_to_goal()
        self.is_reached = True
        return

    # --------------------------  go_to_goal  ---------------------------#

    def go_to_goal(self):

        while self.goal_dist > self.goal_dis_tresh and not rospy.is_shutdown():

            # detect and group
            self.detect_group()

            if self.stop_flag_multi:
                print(self.ind, "stop_flag_multi")
                req = SharePoses2Request()
                req.ind = self.ind
                req.stopped = True
                self.pose_client(req)
                self.v = 0
                self.w = 0
            else:

                # calculate forces
                [f_r, f_theta, phi] = self.forces()

                # calculate velocities
                self.cal_vel(f_r, f_theta, phi)
                self.v_lin.append(self.v)
                self.v_ang.append(self.w)

                if self.stop_flag_obsts or self.stop_flag_robots:
                    print(self.ind, "stop_flag_obsts or stop_flag_robots")
                    # self.v = min(self.v, self.v_min_2)
                    self.v = 0
                    # self.w = 0
                
                if self.stop_flag_full:
                    print(self.ind, "stop_flag_full")
                    req = SharePoses2Request()
                    req.ind = self.ind
                    req.stopped = True
                    self.pose_client(req)
                    self.v = 0
                    self.w = 0

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel_pub.publish(move_cmd)

            # result
            self.path_x.append(round(self.r_x, 3))
            self.path_y.append(round(self.r_y, 3))

            n = 1
            if self.ind==n: print("fm: ", self.stop_flag_multi)
            if self.ind==n: print("f_r", round(f_r, 2), "f_theta", round(f_theta, 2))
            if self.ind==n: print("moving", "v", round(self.v, 2), "w", round(self.w, 2))
            if self.ind==n: print(" ------------------------------------ ")
            self.rate.sleep()

        req = SharePoses2Request()
        req.ind = self.ind
        req.reached = True
        self.pose_client(req)
        self.stop()

    # -----------------------  cal_vel  ----------------------------#

    def cal_vel(self, f_r, f_theta, theta):

        if f_r < 0:
            v = 0
        else:
            v = 1 * self.v_max * ((f_r / self.fix_f)**2 + (f_r / self.fix_f) / 4) + self.v_min_2

        w = 3 * self.w_max * f_theta / self.fix_f

        if  f_r < -1 and abs(w)<0.05:
            w = 1*np.sign(w)

        if (v==0) and abs(w)<0.03:
            v = self.v_min_2*1

        # thresh_theta = np.pi/3
        # w = 4 * self.w_max * theta / (np.pi/6)
        # v = 3 * self.v_max * (1-abs(theta)/thresh_theta)

        # if (v<self.v_min_2) and abs(w)<0.03:
        #     v = self.v_min_2*2

        v = min(v, self.v_max)
        v = max(v, self.v_min)
        wa = min(abs(w), self.w_max)
        w = wa * np.sign(w)

        self.v = v
        self.w = w

    # -----------------------  forces  ----------------------------#

    def forces(self):

        f_r = 0
        f_theta = 0
        self.is_multi = False
        self.near_obst = False
        self.near_robots = False
        self.stop_flag_full = False
        self.stop_flag_obsts = False
        self.stop_flag_robots = False
    
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]

        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]

        theta = np.arctan2(f_theta, f_r)
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        self.viz_arrow([f_r, f_theta], False)

        # self.phis.append(phi)
        # self.force_or.append(self.obs_f[0])
        # self.force_ot.append(self.obs_f[1])
        # self.force_tr.append(self.target_f[0])
        # self.force_tt.append(self.target_f[1])

        return [f_r, f_theta, phi]

    # -----------------------  detect_group  ----------------------------#

    def detect_group(self):

        #
        c_r = 2.5
        is_goal_close = False
        self.stop_flag_multi = False

        #
        groups = []
        AD_h_rR = []
        new_robots = []
        multi_robots = []
        multi_robots_vis = []
        robots_inds = []
        robots_inds_f = {}
        self.new_robots = []

        #
        self.detect_obsts()

        # get data
        req_poses = SharePoses2Request()
        req_poses.ind = self.ind
        req_poses.update = False
        req_poses.stopped = False
        resp_poses = self.pose_client(req_poses)
        robots_x = resp_poses.x
        robots_y = resp_poses.y
        robots_h = resp_poses.heading
        robots_stopped = resp_poses.stopp
        robots_reached = resp_poses.reached
        robots_priority = resp_poses.priority
        
        goal_dist = self.distance(self.r_x, self.r_y, self.goal_x, self.goal_y)
        if (goal_dist < (c_r*self.robot_start_d)):
            is_goal_close = True

        # get indices of robots in proximity circle
        for i in range(resp_poses.count):
            # rR
            dx = (robots_x[i] - self.r_x)
            dy = (robots_y[i] - self.r_y)
            d_rR = np.sqrt(dx**2 + dy**2)
            theta_rR = np.arctan2(dy, dx)
            ad_h_rR = self.angle_diff(self.r_h, theta_rR)
            ad_h_rR_abs = abs(ad_h_rR)
            ad_H_Rr = self.angle_diff(robots_h[i], (theta_rR - np.pi))
            ad_H_Rr_abs = abs(ad_H_Rr)
            AD_h_rR.append(ad_h_rR)

            if (d_rR > (c_r * self.robot_start_d)):
                continue
            
            # if (not robots_reached[i]) or (d_rR < (1 * self.robot_start_d)):                
            # if (d_rR < (1 * self.robot_start_d)) or ((not robots_reached[i]) or (ad_h_rR_abs < np.pi/2 or ad_H_Rr_abs < np.pi/2)):                
            
            if (not robots_reached[i]) or (d_rR < (1 * self.robot_start_d)):                
                robots_inds.append(i)

            #### individual robots
            if (d_rR<(1 * self.robot_start_d)):
                nr = NewRobots()
                nr.d = d_rR
                nr.x= robots_x[i]
                nr.y= robots_y[i]
                nr.H = robots_h[i]
                nr.h_rR = ad_h_rR
                nr.theta_rR = theta_rR
                nr.p = robots_priority[i]>0
                nr.stop = robots_stopped[i]
                nr.reached = robots_reached[i]
                rc = self.robot_prec_d
                if robots_reached[i]:
                    XY = self.eval_obst(robots_x[i], robots_y[i], self.robot_prec_d, d_rR)
                nr.r_prec = rc
                nr.r_half = 1.5 * rc
                nr.r_start = 2.0 * rc
                nr.z = 4 * self.fix_f * rc**4
                new_robots.append(nr)
                if robots_reached[i]: 
                    for xy in XY:
                        nr.x = xy[0]
                        nr.y = xy[1]
                        new_robots.append(nr)
                        multi_robots_vis.append(nr)
        
        # if there is none robots in proximity
        if len(robots_inds)==0:
            return
        
        # generate robots_inds_f (neighbor robots in proximity circle)
        if (not is_goal_close):
            robots_inds_2 = robots_inds[:]
            while (len(robots_inds_2)>0):
                p = robots_inds_2.pop(0)
                robots_inds_f[p] = [p]
                if len(robots_inds_2)==0:
                    break
                for ind_j in robots_inds_2:
                    if not (robots_reached[p] and robots_reached[ind_j]):
                        dist = self.distance(robots_x[p], robots_y[p], robots_x[ind_j], robots_y[ind_j])
                        if (dist<(self.robot_prec_d*2.2)):     ##### robot_start_d robot_prec_d
                            robots_inds_f[p].append(ind_j)

            # detect groups 
            robots_inds_3 = robots_inds[:]
            while len(robots_inds_3)>0:
                groups.append([])
                p = robots_inds_3.pop(0)
                gset = set(robots_inds_f[p])
                robots_inds_4 = robots_inds_3[:]
                for ind_j in robots_inds_4:
                    nset = set(robots_inds_f[ind_j])
                    if len(gset.intersection(nset))>0:
                        gset = gset.union(nset)
                        robots_inds_3.remove(ind_j)
                groups[-1] = list(gset)

        # groups - new_robots -----------------------------------
        for g in groups:
            if len(g)>1:
                self.is_multi = True
                nr = NewRobots()
                nr.big = True
                is_robot_in = False
                is_target_in = False

                # priorities
                P = [robots_priority[i]>0 for i in g]

                # polygon
                is_g2 = True
                if len(g)>2:
                    point_robot = Point(self.r_x, self.r_y)
                    point_target = Point(self.goal_x, self.goal_y)
                    polys_points = [Point(robots_x[i], robots_y[i]) for i in g if (not robots_reached[i])]
                    if (len(polys_points)>2):
                        is_g2 = False
                        mpt = MultiPoint([shape(p) for p in polys_points])
                        mp = mpt.convex_hull
                        mp_bound = mp.boundary.coords
                        mpc = mp.centroid.coords[0]
                        is_robot_in = mp.contains(point_robot)
                        is_target_in = mp.contains(point_target)
                        self.vs.robot_poly([[], mp_bound], self.ns)

                        # get the minimum bounding circle of the convex hull
                        mbr = mp.minimum_rotated_rectangle
                        circum_center = mbr.centroid.coords[0]
                        # calculate the radius of the circumscribed circle
                        radius = mbr.exterior.distance(mbr.centroid)
                        xc = circum_center[0]
                        yc = circum_center[1]
                        rc = 2.5*radius  #+ self.robot_r + self.prec_d
                        rc = max(rc, 2*self.robot_prec_d)
                
                # if robot is in the polygon
                if (not is_target_in) and is_robot_in:
                    self.stop_flag_multi = True
                    return
                
                # robot is not in the polygon, detect """triangle"""
                if is_g2:
                    ad = [AD_h_rR[i] for i in g]
                    a_min = g[np.argmin(ad)]
                    a_max = g[np.argmax(ad)]

                    x1 = robots_x[a_min]
                    x2 = robots_x[a_max]
                    y1 = robots_y[a_min]
                    y2 = robots_y[a_max]
                    dx = x2-x1
                    dy = y2-y1 
                    theta = np.arctan2(dy, dx)
                    d12 = self.distance(x1, y1, x2, y2)

                    xx1 = x1 + (d12/np.sqrt(3)) * np.cos(theta+np.pi/6) 
                    yy1 = y1 + (d12/np.sqrt(3)) * np.sin(theta+np.pi/6)
                    xx2 = x1 + (d12/np.sqrt(3)) * np.cos(theta-np.pi/6) 
                    yy2 = y1 + (d12/np.sqrt(3)) * np.sin(theta-np.pi/6)
                    dd1 = self.distance(xx1, yy1, self.r_x, self.r_y) 
                    dd2 = self.distance(xx2, yy2, self.r_x, self.r_y)
                    if (dd1<dd2):
                        xc = xx2
                        yc = yy2
                    else:
                        xc = xx1
                        yc = yy1
                    rc = d12/np.sqrt(2)      # /np.sqrt(3) d12

                # 
                d_tc = self.distance(self.goal_x, self.goal_y, xc, yc)
                if (d_tc<rc):
                    is_target_in = True
                    continue

                # r_c, r_R
                dx = xc - self.r_x
                dy = yc - self.r_y
                d_rR = np.sqrt(dx**2 + dy**2)
                theta_rR = np.arctan2(dy, dx)
                ad_h_rR = self.angle_diff(self.r_h, theta_rR)

                nr.x = xc
                nr.y = yc
                nr.d = d_rR
                nr.h_rR = ad_h_rR
                nr.theta_rR = theta_rR
                if any(P): nr.p = True
                nr.r_prec = rc + self.robot_r + self.prec_d
                # nr.r_prec = self.eval_obst(xc, yc, nr.r_prec, d_rR)
                nr.r_half = 1.5 * nr.r_prec
                nr.r_start = 2.0 * nr.r_prec
                nr.z = 4 * self.fix_f * nr.r_prec**4
                new_robots.append(nr)
                multi_robots.append(nr)

        if len(multi_robots)>1:
            rcs = [mr.r_prec for mr in multi_robots]
            rcs = np.array(rcs)
            max_ind = np.argmax(rcs)
            multi_robots = [multi_robots[max_ind]]
            multi_robots_vis.append(multi_robots[0])
        elif len(multi_robots)==1:
            multi_robots_vis.append(multi_robots[0])

        self.new_robots = new_robots
        self.vs.robot_data(multi_robots_vis, self.ns)
        return


    def detect_obsts(self):
        f_obsts_inds = []
        for oi in self.obs_ind_main:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            do = self.distance(xo, yo, self.r_x, self.r_y)
            if do<self.obst_start_d:
                f_obsts_inds.append(oi)
        self.f_obsts_inds = f_obsts_inds

    def eval_obst(self, xc, yc, rc, d_rR):
        xy = []
        # ros = [rc]
        for oi in self.obs_ind_main:
            xo = self.obs_x[oi]
            yo = self.obs_y[oi]
            d_Ro = self.distance(xo, yo, xc, yc)
            d_ro = self.distance(xo, yo, self.r_x, self.r_y)
            if True: #d_rR>rc: #d_ro<d_rR and 
                if (rc>(d_Ro-self.obst_prec_d)) and rc<(d_Ro+self.obst_prec_d):
                    # ros.append(d_Ro+self.obst_prec_d*1)
                    x = (xo+xc)/2
                    y = (yo+yc)/2
                    xy.append([x, y])

        # if ros!=[]:
        #     do_max = max(ros)
        #     rc = do_max
        return xy  #rc

    # -----------------------  f_target  ----------------------------#

    def f_target(self):
        # r_g
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_dist = np.sqrt(dx**2 + dy**2)
        # f = self.zeta * goal_dist            
        f = self.fix_f
        if goal_dist<0.5:
            f = 2*f
        theta_rg = np.arctan2(dy, dx)
        ad_rg_h = self.angle_diff(theta_rg, self.r_h)
        self.theta_rg = theta_rg
        self.goal_dist = goal_dist
        self.goal_theta = theta_rg
        fx = round(f * np.cos(ad_rg_h), 3)
        fy = round(f * np.sin(ad_rg_h), 3)
        self.target_f = [fx, fy]

    # -----------------------  f_robots  ----------------------------#

    def f_robots(self):
        
        robot_f = [0, 0]
        self.robot_f = [0, 0]
        self.near_robots = False
        new_robots = self.new_robots

        for nr in new_robots:
            nr_force = [0, 0]
            if (not nr.big):
                nr_force = self.compute_robot_force(nr)
                # self.viz_arrow(nr_force)
            else:
                # if (not self.near_robots) and (not self.near_obst):
                nr_force = self.compute_multi_force(nr)

            robot_f[0] += round(nr_force[0], 3)
            robot_f[1] += round(nr_force[1], 3)

        coeff_f = 1
        self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
        self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    # -----------------------  compute_robot_force  ----------------------------#

    def compute_multi_force(self, nr):
        nr_force = [0, 0]
            
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
        ad_Rg_Rr = self.angle_diff(theta_Rg, theta_Rr)
        if abs(ad_Rg_Rr)<np.deg2rad(180-20):
            based_on_goal = True
        target_other_side = False
        if abs(ad_Rg_Rr)>np.pi/5:
            target_other_side = True

        theta_ = 20
        ad_rg_rR = self.angle_diff(self.theta_rg,  nr.theta_rR)
        if based_on_goal:
            coeff = np.sign(ad_rg_rR*nr.h_rR)
        else:
            if abs(nr.h_rR)<np.deg2rad(theta_):
                coeff = np.sign(ad_rg_rR*nr.h_rR)

        angle_turn_r = nr.theta_rR + (np.pi/2+np.pi/8)*np.sign(nr.h_rR)*coeff
        ad_c_h = self.angle_diff(angle_turn_r, self.r_h)
        f3 = f1 + 4
        templ3 = [f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h)]

        if target_other_side:
            if (nr.r_prec<nr.d):
                nr_force = templ3
            # elif (0.8*nr.r_prec<nr.d<nr.r_prec):
            #     # nr_force = templ3
            #     nr_force = [templ3[0]+nr_force[0], templ3[1]+nr_force[1]]

        return nr_force


    def compute_robot_force(self, nr):
        if (nr.d< nr.r_start):
            if (nr.d< nr.r_prec) and (abs(nr.h_rR)<(np.pi/2+np.pi/10)):
                self.stop_flag_robots = True
                if (not nr.reached) and (not nr.stop) and nr.p:
                    self.stop_flag_full = True
                    return [0, 0]

            if (nr.d<nr.r_half):
                self.near_robots = True

            #
            coeff = 1
            templ2 = []
            templ3 = []
            templ3_2 = []
            nr_force = []

            # compute force
            dx = self.goal_x - nr.x
            dy = self.goal_y - nr.y
            theta_Rg = np.arctan2(dy, dx)
            theta_Rr = nr.theta_rR - np.pi
            ad_Rg_Rr = self.angle_diff(theta_Rg, theta_Rr)
            target_other_side = False
            if abs(ad_Rg_Rr)>np.pi/3:
                target_other_side = True

            ad_h_rR = nr.h_rR
            if (abs(ad_h_rR)<(10*np.pi/180)):
                ad_rg_rR = self.angle_diff(self.theta_rg,  nr.theta_rR)
                coeff = np.sign(ad_rg_rR*nr.h_rR)

            R_coeff = 1
            flag_rR = True
            ad_Rr_H = self.angle_diff((nr.theta_rR - np.pi), nr.H)
            ad_rR_h =  self.angle_diff(nr.theta_rR, self.r_h)

            if (ad_Rr_H*ad_rR_h)<0:
                if ad_rR_h<0:
                    flag_rR = False
                    # R_coeff = -1

            angle_turn_R = nr.theta_rR - (np.pi/2+np.pi/8)*np.sign(ad_Rr_H*R_coeff)
            ad_C_h = self.angle_diff(angle_turn_R, self.r_h)
            angle_turn_r = nr.theta_rR + (np.pi/2+np.pi/8)*np.sign(ad_h_rR)*coeff
            ad_c_h = self.angle_diff(angle_turn_r, self.r_h)

            f = ((nr.z * 1) * ((1 / nr.d) - (1 / nr.r_start))**2) * (1 / nr.d)**2

            fl = f + 0
            nr_force = [fl * -np.cos(ad_h_rR), fl * np.sin(ad_h_rR)]

            f2 = f + 2
            f2_2 = f + 4
            templ2 = [f2 * np.cos(ad_C_h), f2 * np.sin(ad_C_h)]
            templ2_2 = [f2_2 * np.cos(ad_C_h), f2_2 * np.sin(ad_C_h)]

            f3 = f + 2
            f3_2 = f + 4 
            templ3 = [f3 * np.cos(ad_c_h), f3 * np.sin(ad_c_h)]
            templ3_2 = [f3_2 * np.cos(ad_c_h), f3_2 * np.sin(ad_c_h)]
            
            # adjust heading
            if target_other_side:
                if (nr.r_half<nr.d<nr.r_start):
                    if (not nr.reached) and (not nr.stop):
                        if (flag_rR and abs(ad_h_rR)<np.pi/2) and (abs(ad_Rr_H)<(np.pi/2)):
                            nr_force = [templ2[0]+nr_force[0], templ2[1]+nr_force[1]]
                    else:
                        if (abs(ad_h_rR)<(np.pi/2)):
                            nr_force = [templ3[0]+nr_force[0], templ3[1]+nr_force[1]]

                elif (nr.r_prec <nr.d<nr.r_half):
                    if (not nr.reached) and (not nr.stop):
                        if (flag_rR and abs(ad_h_rR)<np.pi/2):
                            nr_force = [templ2_2[0]+nr_force[0], templ2_2[1]+nr_force[1]]
                    else:
                        if (abs(ad_h_rR)<(np.pi/2)):
                            nr_force = [templ3_2[0]+nr_force[0], templ3_2[1]+nr_force[1]]
        return nr_force

    # -----------------------  f_obstacle  ----------------------------#

    def f_obstacle(self):
        obs_f = [0, 0]
        self.obs_f = [0, 0]
        self.near_obst = False

        for i in self.f_obsts_inds:
            dy = (self.obs_y[i] - self.r_y)
            dx = (self.obs_x[i] - self.r_x)
            d_ro = np.sqrt(dx**2 + dy**2)

            theta_ro = np.arctan2(dy, dx)
            ad_h_ro = self.angle_diff(self.r_h, theta_ro)

            if (d_ro < self.obst_half_d):
                self.near_obst = True

            # if (d_ro < self.obst_prec_d) and (abs(ad_h_ro)<(np.pi/2)):
            #     self.stop_flag_obsts = True

            dx = self.goal_x - self.obs_x[i]
            dy = self.goal_y - self.obs_y[i]
            theta_og = np.arctan2(dy, dx)
            theta_or = theta_ro - np.pi
            ad_og_or = self.angle_diff(theta_og, theta_or)
            target_other_side = False
            if abs(ad_og_or)>np.pi/4:
                target_other_side = True

            coeff = 1
            theta = 20
            if (abs(ad_h_ro)<(theta*np.pi/180)):
                ad_rg_ro = self.angle_diff(self.theta_rg,  theta_ro)
                coeff = np.sign(ad_rg_ro*ad_h_ro)
            # angle_turn_o = theta_ro + (np.pi/2)*np.sign(ad_h_ro)
            # ad_c_o = self.angle_diff(angle_turn_o, self.r_h)
            angle_turn_t = theta_ro + (np.pi/2)*np.sign(ad_h_ro)*coeff
            ad_c_t = self.angle_diff(angle_turn_t, self.r_h)


            f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
            o_force = [f * -np.cos(ad_h_ro), f * np.sin(ad_h_ro)]

            # fo = f + 2
            # templo = [fo * np.cos(ad_c_o), fo * np.sin(ad_c_o)]
            
            ft = f + 2
            templt = [ft * np.cos(ad_c_t), ft * np.sin(ad_c_t)]

            if target_other_side:
                if (self.obst_prec_d<d_ro<self.obst_half_d):
                        o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]
                elif (self.obst_prec_d<d_ro):
                    if (abs(ad_h_ro)<np.pi/2):
                            o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]

            obs_f[0] += round(o_force[0], 3)
            obs_f[1] += round(o_force[1], 3)

        coeff_f = 1
        self.obs_f[0] += round(obs_f[0] * coeff_f, 3)
        self.obs_f[1] += round(obs_f[1] * coeff_f, 3)

    # ------------------------- check_topic -- get_odom  ------------------------------------#

    def check_topic(self):
        self.topic_msg = None
        rospy.loginfo(self.ns + " apf_motion, checking topic ...")
        while self.topic_msg is None:
            try:
                self.topic_msg = rospy.wait_for_message(self.topic, self.topic_type, timeout=3.0)
                rospy.logdebug(self.ns + " apf_motion, current topic is ready!")
            except:
                rospy.loginfo(self.ns + " apf_motion, current topic is not ready yet, retrying ...")

        position = self.topic_msg.pose.pose.position
        quaternion = self.topic_msg.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_h = orientation[2]
        return self.topic_msg

    def get_odom(self, odom):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_h = orientation[2]

    # ---------------------------------------------------------#

    def map(self):
        # robot target:
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt
        # obstacles:
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y
        self.obs_count = self.model.obst.count
        self.obs_ind_main = [i for i in range(self.model.obst.count)]

    # [-pi pi]
    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2 * np.pi) - np.pi
        return theta_mod
    
    # [0 2*pi]
    def mod_angle(self, theta):
        if theta < 0:
            theta = theta + 2 * np.pi
        elif theta > 2 * np.pi:
            theta = theta - 2 * np.pi
        return theta
    
    def angle_diff(self, a1, a2):
        ad = a1 - a2
        ad = np.arctan2(np.sin(ad), np.cos(ad))
        return ad

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    
    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel_pub.publish(Twist())
            self.rate.sleep()
            t += 1

    def shutdown_hook(self):
        print("shutting down from apf_motion")
        self.stop()
    
    def viz_arrow(self, nr_force, ip=True):
        n=1
        if self.ind==n: 
            theta = np.arctan2(nr_force[1], nr_force[0])
            theta = np.arctan2(np.sin(theta), np.cos(theta))
            theta = self.r_h + theta
            if ip:
                self.vs.arrow(self.r_x, self.r_y, theta)
            else:
                self.vs.arrow_f(self.r_x, self.r_y, theta)

    # --------------------------------------------------#

    def tensor_force(self):
        import matplotlib.pyplot as plt

        # Define the center coordinates and radius of the circle
        center_x, center_y = self.obs_x[0], self.obs_y[0]
        radius = self.obst_prec_d/5
        radius2 = self.obst_prec_d+0.1

        # # Define the range of x and y values
        x_min, x_max = self.obs_x[0]-1, self.obs_x[0]+1    # self.model.map.x_min, self.model.map.x_max
        y_min, y_max = self.obs_y[0]-1, self.obs_y[0]+1    # self.model.map.y_min, self.model.map.y_max
        x_step, y_step = 0.02, 0.02

        # Create the grid of x and y values
        x = np.arange(x_min, x_max + x_step, x_step)
        y = np.arange(y_min, y_max + y_step, y_step)
        X, Y = np.meshgrid(x, y, indexing='ij')

        # Compute the distance of each point from the center of the circle
        dist = np.sqrt((X - center_x)**2 + (Y - center_y)**2)

        # Create a Boolean mask indicating whether each point is outside the circle
        outside_circle1 = radius < dist
        outside_circle2 = dist < radius2
        outside_circle = np.logical_and(outside_circle1, outside_circle2)

        X[outside_circle == False] = np.nan
        Y[outside_circle == False] = np.nan

        X_outside = X
        Y_outside = Y

        # 
        # r_min, r_max = self.obst_prec_d/2, self.obst_prec_d+0.01 + 2
        # t_min, t_max = 0, np.pi*2
        # r_n, t_n = 10, 100
        # r = np.linspace(r_min, r_max, r_n)
        # t = np.linspace(t_min, t_max, t_n)

        # R, T = np.meshgrid(r, t, indexing='ij')
        # X_outside = center_x + R*np.cos(T)
        # Y_outside = center_y + R*np.sin(T)

        Fx = np.zeros(X_outside.shape)
        Fy = np.zeros(Y_outside.shape)
        print(X_outside.shape, Y_outside.shape)
        
        for i in range(X_outside.shape[0]):
            for j in range(X_outside.shape[1]):
                if X_outside[i,j]==np.nan or Y_outside[i,j]==np.nan:
                    Fx[i,j], Fy[i,j] =  np.nan, np.nan
                else:
                    Fx[i,j], Fy[i,j] =  self.f_obstacle_tensor(X_outside[i,j], Y_outside[i,j])

        # # Compute the force components (Fx and Fy) based on the partial derivatives of f(x, y)
        # Fx, Fy = self.f_obstacle_tensor(x, y)

        # Compute the magnitude of the force vectors
        force_magnitude = np.sqrt(Fx**2 + Fy**2)

        # Create the force tensor by combining the force components (Fx and Fy) into a 2x2 matrix
        force_tensor = np.stack((Fx, Fy), axis=-1)

        # Normalize the force vectors to have a constant length of 0.5
        normalized_force_tensor = force_tensor / force_magnitude[..., None] * 0.5

        # Plot the force vectors using the quiver function with color based on the magnitude
        plt.quiver(X_outside,
                Y_outside,
                force_tensor[..., 0],
                force_tensor[..., 1],
                force_magnitude,
                cmap="coolwarm")
        plt.colorbar()
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Force Tensor Illustration")
        plt.show()

    def f_obstacle_tensor(self, x, y):
        r_h = 0
        obs_f = [0, 0]
        f_obsts_inds = [0]
        for i in f_obsts_inds:
            dy = (self.obs_y[i] - y)
            dx = (self.obs_x[i] - x)
            d_ro = np.sqrt(dx**2 + dy**2)

            theta_ro = np.arctan2(dy, dx)
            ad_h_ro = self.angle_diff(r_h, theta_ro)
            
            if (d_ro < self.obst_prec_d) and (abs(ad_h_ro)<(np.pi/2)):
                self.stop_flag_robots = True

            coeff = 1
            if (abs(ad_h_ro)<(10*np.pi/180)):
                ad_rg_ro = self.angle_diff(self.theta_rg,  theta_ro)
                coeff = np.sign(ad_rg_ro*ad_h_ro)
            angle_turn_t = theta_ro + (np.pi/2)*np.sign(ad_h_ro)*coeff
            ad_c_t = self.angle_diff(angle_turn_t, r_h)

            theta_or = theta_ro + np.pi 
            f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
            o_force = [f * np.cos(theta_or), f * np.sin(theta_or)]
            
            ft = f + 2
            templt = [ft * np.cos(angle_turn_t), ft * np.sin(angle_turn_t)]

            # if (self.obst_prec_d<d_ro):
            #     if (abs(ad_h_ro)<np.pi/2):
            #             o_force = [templt[0]+o_force[0], templt[1]+o_force[1]]

            obs_f[0] += round(o_force[0], 4)
            obs_f[1] += round(o_force[1], 4)

        return obs_f[0], obs_f[1]


# # to do:
# near_obstacles
# if multiple multi_robot_circle, only the closest one.
# obstalce - robot join