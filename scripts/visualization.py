#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class Viusalize:
    def __init__(self, model):

        self.rate = rospy.Rate(10)

        # obstacles
        self.obs_count = model.obst.count
        self.obs_x = model.obst.x
        self.obs_y = model.obst.y
        self.obst_r = 0.11

        # publisher
        self.obst_marker_pub = rospy.Publisher("/obstacles_marker", MarkerArray, queue_size = 2)

        # initialize obst markers and publish
        self.init_visualize()


    def init_visualize(self):
            
            # obstacles
            marker_array = MarkerArray()
            for i in range(self.obs_count):
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.Time.now()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = marker.CYLINDER
                marker.id = i+1

                # Set the scale of the marker
                marker.scale.x = self.obst_r
                marker.scale.y = self.obst_r
                marker.scale.z = 0.4

                # Set the color
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            
                # Set the pose of the marker
                marker.pose.position.x = self.obs_x[i]
                marker.pose.position.y = self.obs_y[i]
                marker.pose.position.z = 0.4/2
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                # marker array
                marker_array.markers.append(marker)

            # publish marler array
            self.obst_markers = marker_array
            self.publish_once_obst_marker()


    def publish_once_obst_marker(self): 
        while not rospy.is_shutdown():
            connections = self.obst_marker_pub.get_num_connections() 
            if connections > 0: 
                self.obst_marker_pub.publish(self.obst_markers) 
                break 
            else: 
                self.rate.sleep()  