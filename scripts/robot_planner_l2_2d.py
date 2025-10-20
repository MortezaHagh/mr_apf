#! /usr/bin/env python

from math import cos, sin
import tf
import rospy
from parameters import Params
from create_model import MRSModel, Robot
from robot_planner_l1_base import RobotPlannerBase


class Planner2D (RobotPlannerBase):
    def __init__(self, model: MRSModel, robot: Robot, params: Params):
        RobotPlannerBase.__init__(self, model, robot, params)
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
        while (not self.mrapf.reached) and (not rospy.is_shutdown()):
            if self.abort:
                self.stop()
                return False

            if not self.data_received:
                rospy.loginfo(f"[planner_2d, {self.ns}]: waiting for fleet data...")
                self.broadcast_transform()
                self.rate.sleep()
                continue

            # Move the robot
            self.next_move()
            self.rate2d.sleep()

            # sim robot movement
            self.move_robot()
            self.broadcast_transform()
        self.stop()
        return True

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
