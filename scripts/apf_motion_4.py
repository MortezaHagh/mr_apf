#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion


class ApfMotion(object):

    def __init__(self, model, robot, init_params):

        # ros
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)

        # preallocation
        self.v_lin = []
        self.v_ang = []
        self.path_x = []
        self.path_y = []
        self.force_tr = []
        self.force_tt = []
        self.force_or = []
        self.force_ot = []
        self.phiis = []

        self.obst_inds = []
        self.robots_inds = []
        self.obsts_dist = []
        self.robots_dist = []
    
        self.stop_flag = False

        # data
        self.model = model
        self.robot = robot

        # parameters
        self.topic_type = Odometry
        self.prioriy = robot.priority
        self.goal_distance = 1000
        self.goal_theta = 0

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
        self.w_min = 0          # init_params.angular_min_speed
        self.w_max = 1.0        # init_params.angular_max_speed
        self.v_min_2 = 0.02     # init_params.linear_min_speed_2

        # settings
        self.zeta = 1                     
        self.fix_f = 4
        self.fix_f2 = 10
        self.obst_r = 0.11
        self.prec_d = 0.06
        self.robot_r = 0.22             
        
        self.obst_prec_d = self.robot_r + self.obst_r + self.prec_d  # 0.57
        self.obst_start_d = self.obst_prec_d*2
        self.obst_z = 4*self.fix_f*self.obst_prec_d**4

        self.robot_prec_d = 2*self.robot_r + self.prec_d  # 0.64
        self.robot_start_d = self.robot_prec_d*2
        self.robot_z = 4 * self.fix_f*self.robot_prec_d**4

        self.w_coeff = 1                        # init_params.w_coeff      # angular velocity coeff
        self.dis_tresh = init_params.dis_tresh  # distance thresh to finish
        self.theta_thresh = 30 * np.pi / 180    # init_params.theta_thresh  # for velocity calculation


        # map: target and obstacles coordinates
        self.map()

        # /cmd_vel puplisher
        self.cmd_vel = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)

        # listener
        self.check_topic()
        rospy.Subscriber(self.topic, self.topic_type, self.get_odom)

        # pose service client
        rospy.wait_for_service(self.pose_srv_name)
        self.pose_client = rospy.ServiceProxy(self.pose_srv_name, SharePoses2)

        # execute goal
        self.exec_cb()

    # --------------------------  exec_cb  ---------------------------#

    def exec_cb(self):
        # move
        self.go_to_goal()
        self.is_reached = True
        return

    # --------------------------  go_to_goal  ---------------------------#

    def go_to_goal(self):
        while self.goal_distance > self.dis_tresh and not rospy.is_shutdown():
            
            # detect and group
            self.detect_group()
            
            # # calculate forces
            # [f_r, f_theta, phi, stop_flag] = self.forces()

            # # calculate velocities
            # self.cal_vel(f_r, f_theta, phi)
            # self.v_lin.append(self.v)
            # self.v_ang.append(self.w)

            # if stop_flag:
            #     self.v = 0
            #     self.w = 0

            # # publish cmd
            # move_cmd = Twist()
            # move_cmd.linear.x = self.v
            # move_cmd.angular.z = self.w
            # self.cmd_vel.publish(move_cmd)

            # # result
            # self.path_x.append(round(self.r_x, 3))
            # self.path_y.append(round(self.r_y, 3))

            self.rate.sleep()

        req = SharePoses2Request()
        req.ind = self.ind
        req.reached = True
        resp = self.pose_client(req)
        self.stop()

    # -----------------------  cal_vel  ----------------------------#

    def cal_vel(self, f_r, f_theta, theta):

        if f_r < 0:
            v = 0
        else:
            v = 1 * self.v_max * ((f_r / self.fix_f)**2 + (f_r / self.fix_f) / 4) + self.v_min_2

        w = 1 * self.w_max * f_theta / self.fix_f

        # if (v==0) and abs(w)<3*np.pi/180:
        #     v = self.v_min_2

        v = min(v, self.v_max)
        v = max(v, self.v_min)
        wa = min(abs(w), self.w_max)
        w = wa * np.sign(w)

        self.v = v
        self.w = w

    # -----------------------  forces  ----------------------------#

    def forces(self):
        self.f_target()
        f_r = self.target_f[0]
        f_theta = self.target_f[1]

        self.f_obstacle()
        f_r += self.obs_f[0]
        f_theta += self.obs_f[1]

        self.f_robots()
        f_r += self.robot_f[0]
        f_theta += self.robot_f[1]

        phi = np.arctan2(f_theta, f_r)
        theta = phi
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        # print(f_r, f_theta)
        # print(" ------------ ")

        self.force_tr.append(self.target_f[0])
        self.force_tt.append(self.target_f[1])
        self.force_or.append(self.obs_f[0])
        self.force_ot.append(self.obs_f[1])
        self.phiis.append(phi)

        return [f_r, f_theta, phi, self.stop_flag]
    
    # -----------------------  detect_group  ----------------------------#

    def detect_group(self):
        
        # self.obst_inds = []
        # self.obsts_dist = []
        self.robots_inds = []
        self.robots_dist = []
        self.robots_pd = []
        self.robots_sd = []

        groups = []
        robots_inds_f = {}

        # for i in range(self.obs_count):
        #     dy = -(self.obs_y[i] - self.r_y)
        #     dx = -(self.obs_x[i] - self.r_x)
        #     d_ro = np.sqrt(dx**2 + dy**2)
        #     if d_ro > self.obst_start_d:
        #         continue
        #     self.obst_inds.append(i)
        #     self.obsts_dist.append(d_ro)


        # get indices of robots in proximity circle
        req_poses2 = SharePoses2Request()
        req_poses2.update = False
        req_poses2.ind = self.ind
        resp_poses2 = self.pose_client(req_poses2)
        robots_x = resp_poses2.x
        robots_y = resp_poses2.y
        for i in range(resp_poses2.count):
            dx = (robots_x[i] - self.r_x)
            dy = (robots_y[i] - self.r_y)
            d_rr = np.sqrt(dx**2 + dy**2)
            if d_rr > 1 * self.robot_start_d:   #####
                continue
            self.robots_inds.append(i)
        
        # if there is only one or none robots in proximity
        if len(self.robots_inds)==0:
            return
        elif len(self.robots_inds)==1:
            self.robots_x = robots_x[0]
            self.robots_y = robots_y[0]
            self.robots_pd = [self.robot_prec_d]
            self.robots_sd = [self.robot_start_d]
            # return
        
        # generate robots_inds_f (neighbor robots in proximity circle)
        robots_inds_2 = self.robots_inds[:]
        while (len(robots_inds_2)>0):
            p = robots_inds_2.pop(0)
            robots_inds_f[p] = [p]
            if len(robots_inds_2)==0:
                break
            for ind_j in robots_inds_2:
                dx = (robots_x[p] - robots_x[ind_j])
                dy = (robots_y[p] - robots_y[ind_j])
                dist = np.sqrt(dx**2+dy**2)
                if dist<self.robot_start_d:     #####
                    robots_inds_f[p].append(ind_j)

        # if self.ind==1:
        #     print("robots_inds_f", robots_inds_f)
        
        # detect groups
        robots_inds_3 = self.robots_inds[:]
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

        if self.ind==1:
            print(groups)
            print(" -------------- ")
        
        

    # -----------------------  f_target  ----------------------------#

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_distance = np.sqrt(dx**2 + dy**2)
        # f = self.zeta * goal_distance * 4
        f = self.fix_f
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_theta = theta
        self.goal_distance = goal_distance
        fx = round(f * np.cos(angle_diff), 3)
        fy = round(f * np.sin(angle_diff), 3)
        self.target_f = [fx, fy]

    # -----------------------  f_robots  ----------------------------#

    def f_robots(self):
        robot_flag = False
        self.stop_flag = False
        req = SharePoses2Request()
        req.ind = self.ind
        req.update = False
        resp = self.pose_client(req)
        robot_f = [0, 0]
        self.robot_f = [0, 0]
        for i in range(resp.count):
            dx = -(resp.x[i] - self.r_x)
            dy = -(resp.y[i] - self.r_y)
            d_rr = np.sqrt(dx**2 + dy**2)
            theta = np.arctan2(dy, dx)
            angle_diff = theta - self.r_theta
            angle_diff2 = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            if d_rr > 1 * self.robot_start_d:
                continue
            
            # if  d_ro < 2.0 * self.robot_prec_d and resp.priority[i] > 0 and abs(angle_diff2) > np.pi / 2:
            #     self.stop_flag = True
            #     # print(self.ns, "stop")
            #     break

            robot_flag = True
            f = ((self.robot_z * 1) * ((1 / d_rr) - (1 / self.robot_start_d))**2) * (1 / d_rr)**2
            templ = [f * np.cos(angle_diff), f * np.sin(angle_diff)]

            if d_rr<2.0*self.robot_prec_d:
                if abs(angle_diff2)>(np.pi/2):
                    angle_diff3 = np.pi - abs(angle_diff2)
                    coeff_alpha = np.cos(angle_diff3)
                    # templ[1] += (f+3.5)*coeff_alpha*np.sign(np.sin(angle_diff2))

                    goal_theta = self.mod_angle(self.goal_theta)
                    angle_diff4 = theta - goal_theta
                    angle_diff4 = np.arctan2(np.sin(angle_diff4), np.cos(angle_diff4))
                    if angle_diff4*angle_diff2<0:
                        coeff_alpha = -1*coeff_alpha

                    templ[1] += (f+3.0)*coeff_alpha*np.sign(np.sin(angle_diff2))

                # else:
                #     templ[0] = f #+ 2.5
                #     templ[1] = 0

            robot_f[0] += round(templ[0], 3)
            robot_f[1] += round(templ[1], 3)

        coeff_f = 1
        if robot_flag:
            abst_f = np.sqrt((robot_f[0]**2 + robot_f[1]**2))
            # if abst_f>0:
            #     coeff_f = min(abst_f, self.fix_f2) / abst_f

            self.robot_f[0] += round(robot_f[0] * coeff_f, 3)
            self.robot_f[1] += round(robot_f[1] * coeff_f, 3)

    # -----------------------  f_obstacle  ----------------------------#

    def f_obstacle(self):
        obst_flag = False
        self.obs_f = [0, 0]
        obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i] - self.r_y)
            dx = -(self.obs_x[i] - self.r_x)
            d_ro = np.sqrt(dx**2 + dy**2)

            if d_ro > self.obst_start_d:
                continue
            
            obst_flag = True
            theta = np.arctan2(dy, dx)
            theta = self.mod_angle(theta)
            r_theta = self.mod_angle(self.r_theta)
            angle_diff = theta - r_theta
            angle_diff2 = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            f = ((self.obst_z * 1) * ((1 / d_ro) - (1 / self.obst_start_d))**2) * (1 / d_ro)**2
            templ = [f * np.cos(angle_diff2), f * np.sin(angle_diff2)]

            if d_ro<2.0*self.obst_prec_d: 
                if abs(angle_diff2)>(np.pi/2):
                    angle_diff3 = np.pi - abs(angle_diff2)
                    coeff_alpha = np.cos(angle_diff3)
                    
                    goal_theta = self.mod_angle(self.goal_theta)
                    angle_diff4 = theta - goal_theta
                    angle_diff4 = np.arctan2(np.sin(angle_diff4), np.cos(angle_diff4))
                    if angle_diff4*angle_diff2<0:
                        coeff_alpha = -1*coeff_alpha
                    
                    templ[1] += (f+3.2)*coeff_alpha*np.sign(np.sin(angle_diff2))

                else:
                    templ[0] = f + 2.0
                    templ[1] = 0
            
            obs_f[0] += round(templ[0], 3)
            obs_f[1] += round(templ[1], 3)

        coeff_f = 1
        if obst_flag:
            # abst_f = np.sqrt((obs_f[0]**2 + obs_f[1]**2))
            # if abst_f>0:
            #     coeff_f = min(abst_f, self.fix_f2) / abst_f

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
        self.r_theta = orientation[2]
        return self.topic_msg

    def get_odom(self, odom):
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_theta = orientation[2]

    # ----------------  get_robot -- map -- modify_angle -- shutdown_hook -------------------#

    def map(self):

        # robot target
        self.goal_x = self.robot.xt
        self.goal_y = self.robot.yt

        # obstacles
        self.obs_count = self.model.obst.count
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y

        # dist o-t
        self.d_ot = []
        for i in range(self.obs_count):
            xo = self.obs_x[i]
            yo = self.obs_y[i]
            d = np.sqrt((xo-self.goal_x)**2 + (yo-self.goal_y)**2)
            self.d_ot.append(d)


    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2 * np.pi) - np.pi
        return theta_mod

    def stop(self):
        t = 0
        while t < 5:
            self.cmd_vel.publish(Twist())
            self.rate.sleep()
            t += 1

    def shutdown_hook(self):
        print("shutting down from apf_motion")
        self.stop()

    def mod_angle(self, theta):
        if theta < 0:
            theta = theta + 2 * np.pi
        elif theta > 2 * np.pi:
            theta = theta - 2 * np.pi
        return theta
