#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from apf.srv import MyPose, MyPoseRequest
from tf.transformations import euler_from_quaternion
from apf.msg import InitRobotAction, InitRobotResult, InitRobotFeedback


class InitRobotAcion(object):
    def __init__(self, init_params, model):

        # ros
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # preallocation
        self.v_lin = []
        self.v_ang = []
        self.path_x = []
        self.path_y = []
        self.force_r = []
        self.force_t = []
        self.res = InitRobotResult()
        self.feedback = InitRobotFeedback()

        # data
        self.model = model
        self.ind = init_params.id
        self.action_name = init_params.action_name
        
        # parameters vel
        self.v = 0
        self.w = 0
        self.v_min = init_params.linear_min_speed
        self.v_max = init_params.linear_max_speed
        self.w_min = init_params.angular_min_speed
        self.w_max = init_params.angular_max_speed

        # parameters & settings
        self.topic_type = Odometry
        self.zeta = init_params.zeta
        self.robot_r = init_params.robot_r
        self.w_coeff = init_params.w_coeff
        self.dis_tresh = init_params.dis_tresh
        self.f_r_min = init_params.f_r_min #
        self.f_r_max = init_params.f_r_max #
        self.f_theta_min = init_params.f_theta_min #
        self.f_theta_max = init_params.f_theta_max #
        self.theta_thresh = init_params.theta_thresh
        self.obs_effect_r = init_params.obs_effect_r
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
        rospy.wait_for_message(self.topic, self.topic_type, timeout=3.0)

        # # pose service client
        # rospy.wait_for_service(self.pose_srv_name)
        # self.pose_client = rospy.ServiceProxy(self.pose_srv_name, MyPose)

        # action
        self.ac_ = actionlib.SimpleActionServer(self.action_name, InitRobotAction, self.exec_cb)
        self.ac_.start()

    # --------------------------  exec_cb  ---------------------------#

    def exec_cb(self, goal):

        # move
        self.go_to_goal()

        # result
        self.res.result = True
        self.res.path_x = self.path_x
        self.res.path_y = self.path_y
        self.ac_.set_succeeded(self.res)
        return

    # --------------------------  go_to_goal  ---------------------------#

    def go_to_goal(self):
        while self.goal_distance > self.dis_tresh and not rospy.is_shutdown():
            # calculate forces
            [f_r, f_theta, phi] = self.forces()
            self.force_r.append(f_r)
            self.force_t.append(f_theta)

            # calculate velocities
            self.cal_vel(f_r, f_theta, phi)
            self.v_lin.append(self.v)
            self.v_ang.append(self.w)

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel.publish(move_cmd)
            
            # result
            self.path_x.append(self.r_x)
            self.path_y.append(self.r_y)

            # # feedback
            # self.feedback.path = [self.r_x, self.r_y]
            # self.ac_.publish_feedback(self.feedback)

            self.rate.sleep()
        
        self.stop()


    # -----------------------  cal_vel  ----------------------------#

    def cal_vel(self, f_r, f_theta, theta):

        if abs(theta)>self.theta_thresh:
            v = 0 + self.v_min/10
            w = self.w_max * np.sign(theta)
        else:
            v = self.v_max * (1- (abs(theta)/self.theta_thresh))**2 + self.v_min/10
            w = theta * self.w_coeff * 0.5
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

        # self.f_robots()
        # f_r += self.robot_f[0]
        # f_theta += self.robot_f[1]

        phi = np.arctan2(f_theta, f_r)
        phi = round(phi, 4)
        return [f_r, f_theta, phi]

    def f_robots(self):
        req = MyPoseRequest()
        req.ind = self.ind
        resp = self.pose_client(req.ind)
        self.robot_f = [0, 0]
        for i in range(resp.count):
            dx = resp.x[i]-self.r_x
            dy = resp.y[i]-self.r_y
            dy = -dy
            dx = -dx
            d_ro = np.sqrt(dx**2+dy**2)
            if d_ro >= self.robot_r:
                f = 0
                theta = 0
            else:
                f = ((self.zeta*1)*((1/d_ro)-(1/self.obs_effect_r))**2)*(1/d_ro)**2
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                self.robot_f[0] += round(templ[0], 2)
                self.robot_f[1] += round(templ[1], 2)

    def f_target(self):
        dx = self.goal_x - self.r_x
        dy = self.goal_y - self.r_y
        goal_distance = np.sqrt(dx**2+dy**2)
        # f = self.zeta * goal_distance
        f = 1.5
        theta = np.arctan2(dy, dx)
        angle_diff = theta - self.r_theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        self.goal_distance = goal_distance
        fx = round(f*np.cos(angle_diff), 2)
        fy = round(f*np.sin(angle_diff), 2)
        self.target_f = [fx, fy]

    def f_obstacle(self):
        self.obs_f = [0, 0]
        for i in range(self.obs_count):
            dy = -(self.obs_y[i]-self.r_y)
            dx = -(self.obs_x[i]-self.r_x)
            d_ro = np.sqrt(dx**2+dy**2)
            if d_ro >= self.obs_effect_r:
                f = 0
                theta = 0
            else:
                f = ((self.zeta*1)*((1/d_ro)-(1/self.obs_effect_r))**2)*(1/d_ro)**2  # mh 100
                theta = np.arctan2(dy, dx)
                angle_diff = theta - self.r_theta
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                templ = [f*np.cos(angle_diff), f*np.sin(angle_diff)]
                if abs(angle_diff)>np.pi/2:
                    templ[1] +=  abs(templ[0]) * np.sign(templ[1] /2)
                self.obs_f[0] += round(templ[0], 2)
                self.obs_f[1] += round(templ[1], 2)

    # ------------------------- check_topic -- get_odom  ------------------------------#
    def check_topic(self):
        self.topic_msg = None
        rospy.loginfo(self.action_name + " apf, checking topic ...")
        while self.topic_msg is None:
            try:
                self.topic_msg = rospy.wait_for_message(
                    self.topic, self.topic_type, timeout=3.0)
                rospy.logdebug(self.action_name + " apf, current topic is ready!")
            except:
                rospy.loginfo(self.action_name + " apf, current topic is not ready yet, retrying ...")
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
        self.goal_x = self.model.robots[self.ind].xt
        self.goal_y = self.model.robots[self.ind].yt

        # obstacles
        self.obs_count = self.model.obst.count
        self.obs_x = self.model.obst.x
        self.obs_y = self.model.obst.y

    def modify_angle(self, theta):
        theta_mod = (theta + np.pi) % (2*np.pi) - np.pi
        return theta_mod

    def stop(self):
        t = 0
        while t < 10:
            self.cmd_vel.publish(Twist())
            self.rate.sleep()
            t += 1
    
    def shutdown_hook(self):
        print("shutting down from robot action")
        self.stop()
