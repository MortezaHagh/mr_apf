#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker, MarkerArray

class Viusalize:
    def __init__(self, model):

        self.rate = rospy.Rate(10)

        # obstacles
        self.obs_count = model.obst.count
        self.obs_x = model.obst.x
        self.obs_y = model.obst.y
        self.obst_r = 0.11

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
        self.robot_stop_d = self.robot_prec_d
        self.robot_z = 4 * self.fix_f*self.robot_prec_d**4

        # publisher
        self.obst_marker_pub = rospy.Publisher("/obstacles_marker", MarkerArray, queue_size = 2)
        self.obst_prec_pc_pub = rospy.Publisher("/obst_prec", PointCloud, queue_size=10)
        self.obst_start_pc_pub = rospy.Publisher("/obst_start", PointCloud, queue_size=10)

        # initialize obst markers and publish
        self.init_obsts()
        self.init_obsts_prec()
        self.init_obsts_start()
        

    def init_obsts(self):
            
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

    
    def init_obsts_prec(self):
        
        prec_circles = []
        obst_prec_points = []

        thetas = np.linspace(0, np.pi*2, 360)

        for i in range(self.obs_count):
            prec_circles.append([])
            c_x = self.obs_x[i]
            c_y = self.obs_y[i]
            for th in thetas:
                p = Point32()
                p.x = c_x + self.obst_prec_d*np.cos(th)
                p.y = c_y + self.obst_prec_d*np.sin(th)
                prec_circles[-1].append(p)

            obst_prec_points.extend(prec_circles[-1])
        
        obst_prec_pc = PointCloud()
        obst_prec_pc.header.frame_id = "map"
        obst_prec_pc.points = obst_prec_points
        
        # channel = ChannelFloat32()
        # channel.name = "intensities"
        # channel.values = [80 for i in range(len(obst_prec_points))]
        # obst_prec_pc.channels.append(channel)
        
        #
        self.obst_prec_pc = obst_prec_pc
        self.publish_once_obst_prec_points()

    
    def publish_once_obst_prec_points(self): 
        while not rospy.is_shutdown():
            connections = self.obst_prec_pc_pub.get_num_connections() 
            if connections > 0: 
                self.obst_prec_pc_pub.publish(self.obst_prec_pc) 
                break 
            else: 
                self.rate.sleep()
    

    def init_obsts_start(self):
        
        prec_circles = []
        obst_prec_points = []

        thetas = np.linspace(0, np.pi*2, 360)

        for i in range(self.obs_count):
            prec_circles.append([])
            c_x = self.obs_x[i]
            c_y = self.obs_y[i]
            for th in thetas:
                p = Point32()
                p.x = c_x + self.obst_prec_d*np.cos(th)
                p.y = c_y + self.obst_prec_d*np.sin(th)
                prec_circles[-1].append(p)

            obst_prec_points.extend(prec_circles[-1])
        
        obst_prec_pc = PointCloud()
        obst_prec_pc.header.frame_id = "map"
        obst_prec_pc.points = obst_prec_points
        
        # channel = ChannelFloat32()
        # channel.name = "intensities"
        # channel.values = [80 for i in range(len(obst_prec_points))]
        # obst_prec_pc.channels.append(channel)
        
        #
        self.obst_prec_pc = obst_prec_pc
        self.publish_once_obst_prec_points()

    
    def publish_once_obst_prec_points(self): 
        while not rospy.is_shutdown():
            connections = self.obst_prec_pc_pub.get_num_connections() 
            if connections > 0: 
                self.obst_prec_pc_pub.publish(self.obst_prec_pc) 
                break 
            else: 
                self.rate.sleep()
    