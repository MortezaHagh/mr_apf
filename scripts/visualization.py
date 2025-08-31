#! /usr/bin/env python

""" Visualization class for MRAPF """

from typing import List, Dict, Tuple
from array import array
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from parameters import Params
from create_model import MRSModel
from mrapf_classes import ApfRobot


class RvizViusalizer:
    """Rviz visualizer for MRAPF simulation.
    """
    model: MRSModel
    params: Params

    def __init__(self, model: MRSModel):

        rospy.loginfo("[RvizViusalizer]: Initializing RvizViusalizer ...")
        self.params = Params()
        self.model = model
        self.rate = rospy.Rate(10)

        # publishers
        self.obst_marker_pub = rospy.Publisher("/obstacles_marker", MarkerArray, queue_size=2)
        self.obst_prec_pc_pub = rospy.Publisher("/obst_prec", PointCloud, queue_size=10)
        self.obst_start_pc_pub = rospy.Publisher("/obst_start", PointCloud, queue_size=10)
        self.robots_precs_pc_pub = rospy.Publisher("/robots_precs", PointCloud, queue_size=10)
        self.robots_starts_pc_pub = rospy.Publisher("/robots_starts", PointCloud, queue_size=10)
        self.robots_text_pub = rospy.Publisher("/robots_texts", MarkerArray, queue_size=10)
        self.arrow_pub = rospy.Publisher("/arrows", Marker, queue_size=10)
        # self.arrowf_pub = rospy.Publisher("/arrowsf", Marker, queue_size=10)

        # publishers for robots
        self.robots_pubs = {}  # robot prec circles
        self.robots_poly_pubs = {}
        self.robots_poly_pubs_2 = {}
        self.robots_arrow_pubs = {}
        self.add_robot_publishers()

        #
        self.visualize_obstacles()

        # circle thetas
        self.thetas = np.linspace(0, np.pi*2, 180)

        rospy.loginfo("[RvizViusalizer]: RvizViusalizer initialized.")

    def visualize_obstacles(self):
        # initialize obst markers and publish
        self.publish_obsts_markers()
        self.publish_obsts_prec_circles()
        # self.publish_obsts_start_circles()

    def add_robot_publishers(self):
        for ns in self.model.robots_data.ns:
            self.robots_pubs[ns] = rospy.Publisher(ns+"/robot_data", PointCloud, queue_size=10)
            self.robots_poly_pubs[ns] = rospy.Publisher(ns+"/robot_poly", PolygonStamped, queue_size=10)
            self.robots_poly_pubs_2[ns] = rospy.Publisher(ns+"/robot_poly_2", PolygonStamped, queue_size=10)
            self.robots_arrow_pubs[ns] = rospy.Publisher(ns+"/arrowsf", Marker, queue_size=10)

    def update_robot_vizuals(self, xy: Dict[int, List[float]]):
        # self.publish_robots_start_circles(xy)
        self.publish_robots_prec_circles(xy)
        self.publish_robots_texts(xy)

    def publish_obsts_markers(self):
        marker_array = MarkerArray()
        for i in range(self.model.n_obst_orig):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = marker.CYLINDER
            marker.id = i+1
            # Set the scale of the marker
            marker.scale.x = self.params.obst_r
            marker.scale.y = self.params.obst_r
            marker.scale.z = 0.4
            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            # Set the pose of the marker
            marker.pose.position.x = self.model.obsts.x[i]
            marker.pose.position.y = self.model.obsts.y[i]
            marker.pose.position.z = 0.4/2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            # marker array
            marker_array.markers.append(marker)

        # publish marker array
        self.publish_once(self.obst_marker_pub, marker_array, "marker_array ...")

    def publish_obsts_prec_circles(self):
        obst_prec_points = []
        thetas = np.linspace(0, np.pi*2, 360)
        #
        for i in range(self.model.n_obst_orig):
            prec_circles = []
            c_x = self.model.obsts.x[i]
            c_y = self.model.obsts.y[i]
            for th in thetas:
                p = Point32()
                p.x = c_x + self.params.obst_prec_d*np.cos(th)
                p.y = c_y + self.params.obst_prec_d*np.sin(th)
                prec_circles.append(p)
            obst_prec_points.extend(prec_circles)
        #
        obst_prec_pc = PointCloud()
        obst_prec_pc.header.frame_id = "map"
        obst_prec_pc.points = obst_prec_points
        # channel = ChannelFloat32()
        # channel.name = "intensities"
        # channel.values = [80 for i in range(len(obst_prec_points))]
        # obst_prec_pc.channels.append(channel)
        self.publish_once(self.obst_prec_pc_pub, obst_prec_pc, "obst_prec_pc ...")

    def publish_obsts_start_circles(self):
        obst_start_points = []
        thetas = np.linspace(0, np.pi*2, 360)
        #
        for i in range(self.model.n_obst_orig):
            prec_circles = []
            c_x = self.model.obsts.x[i][i]
            c_y = self.model.obsts.y[i][i]
            for th in thetas:
                p = Point32()
                p.x = c_x + self.params.obst_start_d*np.cos(th)
                p.y = c_y + self.params.obst_start_d*np.sin(th)
                prec_circles.append(p)
            obst_start_points.extend(prec_circles)
        #
        obst_start_pc = PointCloud()
        obst_start_pc.header.frame_id = "map"
        obst_start_pc.points = obst_start_points
        self.publish_once(self.obst_start_pc_pub, obst_start_pc, "obst_start_pc ...")

    def publish_robots_prec_circles(self, xy_dict: dict):
        robot_circles = []
        robot_circle = []
        for xy in xy_dict.values():
            c_x = xy[0]
            c_y = xy[1]
            for th in self.thetas:
                p = Point32()
                p.x = c_x + self.params.robot_prec_d*np.cos(th)
                p.y = c_y + self.params.robot_prec_d*np.sin(th)
                robot_circle.append(p)
            robot_circles.extend(robot_circle)
        #
        robots_prec_pc = PointCloud()
        robots_prec_pc.header.frame_id = "map"
        robots_prec_pc.points = robot_circles
        self.publish_once(self.robots_precs_pc_pub, robots_prec_pc, "robots_precs_circles ...")

    def publish_robots_start_circles(self, xyd: dict):
        robot_circles = []
        robot_circle = []
        for xy in xyd.values():
            c_x = xy[0]
            c_y = xy[1]
            for th in self.thetas:
                p = Point32()
                p.x = c_x + self.params.robot_start_d*np.cos(th)
                p.y = c_y + self.params.robot_start_d*np.sin(th)
                robot_circle.append(p)
            robot_circles.extend(robot_circle)
        #
        robots_start_pc = PointCloud()
        robots_start_pc.header.frame_id = "map"
        robots_start_pc.points = robot_circles
        self.publish_normal(self.robots_starts_pc_pub, robots_start_pc, "robots_start_circles ...")

    def publish_robots_texts(self, xyd: dict):
        marker_array = MarkerArray()
        for rid, xy in xyd.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = "r" + str(rid)
            marker.id = rid
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
        self.publish_normal(self.robots_text_pub, marker_array, "robots_texts ...")

    def draw_robot_circles(self, nrs: List[ApfRobot], ns):
        robot_circles = []
        circle = []
        for nr in nrs:
            c_x = nr.x
            c_y = nr.y
            for th in self.thetas:
                p = Point32()
                p.x = c_x + nr.r_prec*np.cos(th)
                p.y = c_y + nr.r_prec*np.sin(th)
                circle.append(p)
            robot_circles.extend(circle)
        #
        robot_prec_pc = PointCloud()
        robot_prec_pc.header.frame_id = "map"
        robot_prec_pc.points = robot_circles
        self.robots_pubs[ns].publish(robot_prec_pc)

    def vizualize_polygon(self, pols_xy: List[Tuple[array, array]], ns: str, rid: int) -> None:
        if len(pols_xy) == 0:
            return
        pol_stamp = PolygonStamped()
        pol_stamp.header.frame_id = "map"
        polygon = Polygon()
        for xy in pols_xy:
            for x, y in zip(xy[0], xy[1]):
                point = Point32()
                point.x = x
                point.y = y
                point.z = 0
                polygon.points.append(point)
        #
        pol_stamp.polygon = polygon
        if rid == 0:
            self.robots_poly_pubs[ns].publish(pol_stamp)
        else:
            self.robots_poly_pubs_2[ns].publish(pol_stamp)

    def visualize_arrow(self, x, y, theta, ns=None):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = marker.ARROW

        q = quaternion_from_euler(0, 0, theta)

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        # Set the scale of the marker
        marker.scale.x = 0.9
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # self.arrowf_pub.publish(marker)
        if ns is None:
            self.arrow_pub.publish(marker)
        else:
            self.robots_arrow_pubs[ns].publish(marker)

    def publish_normal(self, publisher, data, notif):
        publisher.publish(data)
        rospy.loginfo(notif)
        self.rate.sleep()

    def publish_once(self, publisher, data, notif):
        # publisher.publish(data)
        while not rospy.is_shutdown():
            connections = publisher.get_num_connections()
            if connections > 0:
                publisher.publish(data)
                break
            else:
                rospy.loginfo(notif)
                self.rate.sleep()
