#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from apf.srv import SharePoses2, SharePoses2Request
from tf.transformations import euler_from_quaternion


class ApfMotion(object):
    def __init__(self, model, robot, init_params):

        # ros
        self.rate = rospy.Rate(20)
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
        self.stop_flag = False

        # data
        self.model = model
        self.robot = robot
        self.ind = init_params.id
        self.ns = init_params.name_space
        
        # parameters vel
        self.v = 0
        self.w = 0
        self.v_min = 0.02 #init_params.linear_min_speed
        self.v_max = 0.2 #init_params.linear_max_speed
        self.w_min = 0 #init_params.angular_min_speed
        self.w_max = 1.0 #init_params.angular_max_speed

        # parameters & settings
        self.fix_f = 1
        self.prioriy = robot.priority
        self.topic_type = Odometry
        self.zeta = init_params.zeta
        self.robot_r = 0.5 # init_params.robot_r
        self.w_coeff = init_params.w_coeff          # angular velocity coeff
        self.dis_tresh = init_params.dis_tresh      # distance thresh to finish
        # self.f_r_min = init_params.f_r_min #
        # self.f_r_max = init_params.f_r_max #
        # self.f_theta_min = init_params.f_theta_min #
        # self.f_theta_max = init_params.f_theta_max #
        self.theta_thresh = 30*np.pi/180 #init_params.theta_thresh  # for velocity calculation
        self.obs_effect_r = 0.4 #init_params.obs_effect_r
        self.pose_srv_name = init_params.pose_srv_name
        self.goal_distance = init_params.goal_distance

        # map: target and obstacles coordinates
        self.map()

        # /cmd_vel puplisher
        self.cmd_vel = rospy.Publisher(init_params.cmd_topic, Twist, queue_size=5)

        # listener
        self.topic = init_params.lis_topic
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
            # calculate forces
            [f_r, f_theta, phi, stop_flag] = self.forces()

            # calculate velocities
            self.cal_vel(f_r, f_theta, phi)
            self.v_lin.append(self.v)
            self.v_ang.append(self.w)

            if stop_flag:
                self.v = 0 #self.v/3
                self.w = 0 #self.w/3
                # self.stop()
                # continue

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel.publish(move_cmd)
            
            # result
            self.path_x.append(round(self.r_x, 3))
            self.path_y.append(round(self.r_y, 3))

            # # feedback
            # self.feedback.path = [self.r_x, self.r_y]
            # self.ac_.publish_feedback(self.feedback)

            self.rate.sleep()
        
        self.stop()

    # -----------------------  cal_vel  ----------------------------#

    def cal_vel(self, f_r, f_theta, theta):

        # if abs(theta)>self.theta_thresh:
        #     v = 0 + self.v_min/10
        #     w = self.w_max * np.sign(theta)
        # else:
        if self.ind==1:
            print("self.obs_f", self.obs_f)
            print("self.robot_f", self.robot_f)
            print("self.target_f", self.target_f)
            print(self.ind, "f_r:" , f_r, "f_theta", f_theta)
            print("theta2", theta*180/3.14)
            print("================")

        theta2 = abs(theta)
        theta_thresh = 180*np.pi/180 #self.theta_thresh
        v = self.v_max * max(0, (1- (theta2)/theta_thresh))**2 + self.v_min
        w = self.w_max * self.w_coeff * 4 * (theta2/theta_thresh)**1 * np.sign(theta)
        v = min(v, self.v_max)
        v = max(v, 0)
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

        phi = np.arctan2(f_theta, f_r,)
        theta =  phi
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        phi = round(theta, 4)

        self.force_tr.append(self.target_f[0])
        self.force_tt.append(self.target_f[1])
        self.force_or.append(self.obs_f[0])
        self.force_ot.append(self.obs_f[1])
        self.phiis.append(phi)


        return [f_r, f_theta, phi, self.stop_flag]

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_distance = np.sqrt(dx**2+dy**2)
        # f = self.zeta * goal_distance
        f = 5 #self.fix_f 
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_distance = goal_distance
        fx = round(f*np.cos(angle_diff), 3)
        fy = round(f*np.sin(angle_diff), 3)
        self.target_f = [fx, fy]
    

    def f_robots(self):
        self.stop_flag = False
        req = SharePoses2Request()
        req.ind = self.ind
        req.update = False
        resp = self.pose_client(req)
        self.robot_f = [0, 0]
        for i in range(resp.count):
            # heading = resp.heading
            dx = -(resp.x[i]-self.r_x)
            dy = -(resp.y[i]-self.r_y)
            d_ro = np.sqrt(dx**2+dy**2)
            theta = np.arctan2(dy, dx)
            angle_diff = theta - self.r_theta
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            
            robot_r = self.robot_r
            if d_ro > 1*robot_r:  #or abs(angle_diff)<np.pi/2:
                continue
            else:
                if resp.priority[i]>0 and abs(angle_diff)>np.pi/2:
                    self.stop_flag = True
                    break
                
                # if d_ro > robot_r:
                #     f = self.fix_f * (2*robot_r-d_ro)/robot_r           
                # else:
                #     f = self.fix_f + ((self.zeta*1)*((1/d_ro)-(1/robot_r))**2)*(1/d_ro)**2 

                f = ((self.zeta*1)*((1/d_ro)-(1/robot_r))**2)*(1/d_ro)**2 

                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                
                # if angle_diff>0:
                #     phi = theta-np.pi/2
                # else:
                #     phi = theta+np.pi/2
                # angle_diff = phi - self.r_theta
                # angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                
                templ[0] += f*np.cos(angle_diff)
                templ[1] += f*np.sin(angle_diff)   

            self.robot_f[0] += round(templ[0], 3)
            self.robot_f[1] += round(templ[1], 3)

    def f_obstacle(self):
        self.obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i]-self.r_y)
            dx = -(self.obs_x[i]-self.r_x)
            d_ro = np.sqrt(dx**2+dy**2)
            obs_effect_r = 0.6 # self.obs_effect_r
            if d_ro > obs_effect_r:
                continue
            else:
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                
                # if d_ro > obs_effect_r:
                #     f = self.fix_f * (2*obs_effect_r-d_ro)/obs_effect_r
                # else:
                #     f = self.fix_f + ((self.zeta*1)*((1/d_ro)-(1/obs_effect_r))**2)*(1/d_ro)**2 

                f = ((self.zeta*1)*((1/d_ro)-(1/obs_effect_r))**2)*(1/d_ro)**2 

                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                
                # if angle_diff>0:
                #     phi = theta-np.pi/2
                # else:
                #     phi = theta+np.pi/2
                # angle_diff = phi - self.r_theta
                # angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                
                templ[0] += f * np.cos(angle_diff)
                templ[1] += f * np.sin(angle_diff)        

            self.obs_f[0] += round(templ[0], 3)
            self.obs_f[1] += round(templ[1], 3)

    # ------------------------- check_topic -- get_odom  ------------------------------#
    def check_topic(self):
        self.topic_msg = None
        rospy.loginfo(self.ns + " apf_motion, checking topic ...")
        while self.topic_msg is None:
            try:
                self.topic_msg = rospy.wait_for_message(self.topic, self.topic_type, timeout=3.0)
                rospy.logdebug(self.ns + " apf_motion, current topic is ready!")
            except:
                rospy.loginfo(self.ns+ " apf_motion, current topic is not ready yet, retrying ...")
        
        position =  self.topic_msg.pose.pose.position
        quaternion =  self.topic_msg.pose.pose.orientation
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

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2*np.pi) - np.pi
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
        if theta<0:
            theta = theta + 2*np.pi
        elif theta>2*np.pi:
            theta = theta- 2*np.pi
        return theta
        
