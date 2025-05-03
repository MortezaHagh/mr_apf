#! /usr/bin/env python

from math import cos, sin
import tf
import rospy
from geometry_msgs.msg import Twist
from parameters import Params
from create_model import MRSModel, Robot
from robot_planner_l1_base import RobotPlanner


class Planner2D (RobotPlanner):
    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        RobotPlanner.__init__(self, model, robot, params)
        self.rate2d = rospy.Rate(50)

        # Initialize the broadcaster
        self.global_frame = params.global_frame
        self.odom_frame = params.sns + params.odom_frame
        self.local_frame = params.sns + params.local_frame
        self.tf_broadcaster = tf.TransformBroadcaster()

        # init pose
        self.pose.x = robot.xs
        self.pose.y = robot.ys
        self.pose.theta = robot.heading
        self.broadcast_transform()

    def go_to_goal(self):
        while (not self.ap.reached) and (not rospy.is_shutdown()):
            if not self.data_received:
                rospy.loginfo(f"[planner_2d, {self.ns}]: waiting for fleet data...")
                self.broadcast_transform()
                self.rate.sleep()
                continue

            # command *********************************************************
            self.ap.planner_move(self.pose, self.fleet_data)
            self.v = self.ap.v
            self.w = self.ap.w
            self.pd.v.append(self.v)
            self.pd.w.append(self.w)
            self.pd.steps += 1

            # record path
            self.pd.x.append(round(self.pose.x, 3))
            self.pd.y.append(round(self.pose.y, 3))

            # vizualize
            if self.do_viz:
                self.vizualize_force([self.ap.f_r, self.ap.f_theta], False)
                if self.p.method == 2:
                    self.vs.robot_poly(self.ap.mp_bound, self.ns, self.p.rid)
                    self.vs.draw_robot_circles(self.ap.multi_robots_vis, self.ns)

            # publish cmd
            move_cmd = Twist()
            move_cmd.linear.x = self.v
            move_cmd.angular.z = self.w
            self.cmd_vel_pub.publish(move_cmd)

            # check stop
            if self.ap.stopped != self.ap.prev_stopped:
                self.ruc_req.stopped = self.ap.stopped
                self.robot_update_client(self.ruc_req)
                self.ap.prev_stopped = self.ap.stopped

            # log data
            self.log_motion(self.ap.f_r, self.ap.f_theta)
            self.rate2d.sleep()

            # sim robot movement
            self.move_robot()
            self.broadcast_transform()

            #
            if self.ap.reached:
                rospy.loginfo(f"[planner_2d, {self.ns}]: robot reached goal.")

        # reached
        self.ruc_req.stopped = True
        self.ruc_req.reached = True
        self.robot_update_client(self.ruc_req)
        self.stop()
        self.stop()
        rospy.loginfo(f"[planner_2d, {self.ns}]: go_to_goal finished!")

    def move_robot(self):
        self.pose.x += (self.v * self.dt) * cos(self.pose.theta)
        self.pose.y += (self.v * self.dt) * sin(self.pose.theta)
        self.pose.theta += self.w * self.dt

    def broadcast_transform(self):
        self.tf_broadcaster.sendTransform(
            (self.pose.x, self.pose.y, 0),  # Translation
            tf.transformations.quaternion_from_euler(0, 0, self.pose.theta),  # Rotation
            rospy.Time.now(),
            self.local_frame,  # Child frame
            self.odom_frame  # Parent frame
        )
