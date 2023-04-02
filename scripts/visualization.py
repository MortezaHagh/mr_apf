#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker, MarkerArray

class Viusalize:
    def __init__(self, model):

        print("Viusalize started.")

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
        self.robots_precs_pc_pub = rospy.Publisher("/robots_precs", PointCloud, queue_size=10)
        self.robots_starts_pc_pub = rospy.Publisher("/robots_starts", PointCloud, queue_size=10)
        self.robots_text_pub = rospy.Publisher("/robots_texts", MarkerArray, queue_size=10)

        self.robots_pubs = {}
        self.robots_poly_pubs = {}
        self.robots_poly_pubs_2 = {}
        self.add_robot(model)
        # self.robot_data_pub = rospy.Publisher("/robot_data", PointCloud, queue_size=10)

        # initialize obst markers and publish
        self.init_obsts()
        self.init_obsts_prec()
        self.init_obsts_start()

        self.thetas = np.linspace(0, np.pi*2, 180)
        print("Viusalize init done.")
    
    def add_robot(self, model):
        for ns in model.robots_i.ns:
            self.robots_pubs[ns] = (rospy.Publisher(ns+"/robot_data", PointCloud, queue_size=10))
            self.robots_poly_pubs[ns] = (rospy.Publisher(ns+"/robot_poly", PolygonStamped, queue_size=10))
            self.robots_poly_pubs_2[ns] = (rospy.Publisher(ns+"/robot_poly_2", PolygonStamped, queue_size=10))

    def robots_circles(self, xy):
        self.robots_prec_circles(xy)
        # self.robots_starts_circles(xy)
        self.robots_texts(xy)


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
        self.publish_once(self.obst_marker_pub, marker_array, "marker_array ...")

    
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
        self.publish_once(self.obst_prec_pc_pub, obst_prec_pc, "obst_prec_pc ...")
    

    def init_obsts_start(self):
        
        prec_circles = []
        obst_start_points = []

        thetas = np.linspace(0, np.pi*2, 360)

        for i in range(self.obs_count):
            prec_circles.append([])
            c_x = self.obs_x[i]
            c_y = self.obs_y[i]
            for th in thetas:
                p = Point32()
                p.x = c_x + self.obst_start_d*np.cos(th)
                p.y = c_y + self.obst_start_d*np.sin(th)
                prec_circles[-1].append(p)

            obst_start_points.extend(prec_circles[-1])
        
        obst_start_pc = PointCloud()
        obst_start_pc.header.frame_id = "map"
        obst_start_pc.points = obst_start_points
        
        self.publish_once(self.obst_start_pc_pub, obst_start_pc, "obst_start_pc ...")

    
    def robots_prec_circles(self, xyd):
        robots_circles = []
        robot_circle = []
        for k, xy in xyd.items():
            c_x = xy[0]
            c_y = xy[1]
            for th in self.thetas:
                p = Point32()
                p.x = c_x + self.robot_prec_d*np.cos(th)
                p.y = c_y + self.robot_prec_d*np.sin(th)
                robot_circle.append(p)
            robots_circles.extend(robot_circle)
        
        robots_prec_pc = PointCloud()
        robots_prec_pc.header.frame_id = "map"
        robots_prec_pc.points = robots_circles

        self.publish_once(self.robots_precs_pc_pub, robots_prec_pc, "robots_precs_circles ...")


    def robots_starts_circles(self, xyd):
        robots_circles = []
        robot_circle = []
        for k, xy in xyd.items():
            c_x = xy[0]
            c_y = xy[1]
            for th in self.thetas:
                p = Point32()
                p.x = c_x + self.robot_start_d*np.cos(th)
                p.y = c_y + self.robot_start_d*np.sin(th)
                robot_circle.append(p)
            robots_circles.extend(robot_circle)
        
        robots_start_pc = PointCloud()
        robots_start_pc.header.frame_id = "map"
        robots_start_pc.points = robots_circles

        self.publish_once(self.robots_starts_pc_pub, robots_start_pc, "robots_start_circles ...")

    
    def robots_texts(self, xyd):
        marker_array = MarkerArray()
        i = 0
        for k, xy in xyd.items():
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()

            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = "r" + str(i+1)
            marker.id = i
            i = i+1

            # Set the scale of the marker
            marker.scale.z = 0.2

            # Set the color
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        
            # Set the pose of the marker
            marker.pose.position.x = xy[0]
            marker.pose.position.y = xy[1]
            marker.pose.position.z = 0.3
            marker.pose.orientation.w = 1.0

            # marker array
            marker_array.markers.append(marker)

        self.publish_once(self.robots_text_pub, marker_array, "robots_texts ...")


    def robot_data(self, nrs, ns):

        robots_circles = []
        circle = []
        for nr in nrs:
            c_x = nr.x
            c_y = nr.y
            for th in self.thetas:
                p = Point32()
                p.x = c_x + nr.r_prec*np.cos(th)
                p.y = c_y + nr.r_prec*np.sin(th)
                circle.append(p)
            robots_circles.extend(circle)

        robot_prec_pc = PointCloud()
        robot_prec_pc.header.frame_id = "map"
        robot_prec_pc.points = robots_circles
        self.robots_pubs[ns].publish(robot_prec_pc)


    def publish_once(self, publisher, data, notife): 
        publisher.publish(data) 
        # while not rospy.is_shutdown():
        #     connections = publisher.get_num_connections() 
        #     if connections > 0: 
        #         publisher.publish(data) 
        #         break 
        #     else: 
        #         print(notife) 
        #         self.rate.sleep()

    
    def robot_poly(self, pols, ns):
        i = -1
        for po in pols:
            i= i+1
            if len(po)==0:
                continue
            
            pol_stamp = PolygonStamped()
            pol_stamp.header.frame_id = "map"

            polygon = Polygon()
            for p in po:
                point = Point32()
                point.x = p[0]
                point.y = p[1]
                point.z = 0
                polygon.points.append(point)

            pol_stamp.polygon = polygon
            if i==0:
                self.robots_poly_pubs[ns].publish(pol_stamp)
            else:
                self.robots_poly_pubs_2[ns].publish(pol_stamp)

