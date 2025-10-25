#! /usr/bin/env python

""" Visualization class for MRAPF """

from typing import Dict, List, Tuple
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray
from create_model import MRSModel, Obstacle
from mrapf_classes import ApfRobot


class RvizViusalizer:
    """Rviz visualizer for MRAPF simulation.
    """

    def __init__(self, model: MRSModel):

        rospy.loginfo("[RvizViusalizer]: Initializing RvizViusalizer ...")
        self.params = model.params
        self.model = model
        self.rate = rospy.Rate(10)

        # circle thetas
        self.thetas = np.linspace(0, np.pi*2, 180)
        self.circle_points = [(np.cos(th), np.sin(th)) for th in self.thetas]

        # # publishers
        # obstacles publishers
        self.obst_marker_pub = rospy.Publisher("/obstacles_marker", MarkerArray, queue_size=2)
        self.obst_prec_pc_pub = rospy.Publisher("/obst_prec", PointCloud, queue_size=10)
        self.obst_start_pc_pub = rospy.Publisher("/obst_start", PointCloud, queue_size=10)
        # robots publishers
        self.robots_precs_pc_pub = rospy.Publisher("/robots_precs", PointCloud, queue_size=10)
        self.robots_starts_pc_pub = rospy.Publisher("/robots_starts", PointCloud, queue_size=10)
        self.robots_text_pub = rospy.Publisher("/robots_texts", MarkerArray, queue_size=10)
        # arrows publisher
        self.arrow_pub = rospy.Publisher("/arrows", Marker, queue_size=10)
        # self.arrowf_pub = rospy.Publisher("/arrowsf", Marker, queue_size=10)
        # # fake obstacles publishers
        self.fake_obsts_pc_pub = rospy.Publisher("/fake_obstacles", PointCloud, queue_size=10)

        # publishers for robots
        self.robots_circle_pubs = {}  # robot prec circles
        self.robots_poly_pubs = {}
        self.robots_arrow_pubs = {}
        self.add_robot_publishers()

        # visualize obstacles in the map at the beginning
        self.visualize_map_obstacles()

        rospy.loginfo("[RvizViusalizer]: RvizViusalizer initialized.")

    def add_robot_publishers(self):
        for r in self.model.robots:
            ns = r.ns
            self.robots_circle_pubs[ns] = rospy.Publisher(ns+"/robot_data", PointCloud, queue_size=10)
            self.robots_poly_pubs[ns] = rospy.Publisher(ns+"/robot_poly", PolygonStamped, queue_size=10)
            self.robots_arrow_pubs[ns] = rospy.Publisher(ns+"/arrowsf", Marker, queue_size=10)

    def update_robot_visuals(self, xy: Dict[int, List[float]]):
        # self.publish_robots_start_circles(xy)
        self.publish_robots_prec_circles(xy)
        self.publish_robots_texts(xy)

    def visualize_map_obstacles(self):
        # initialize obst markers and publish
        self.publish_obsts_markers()
        self.publish_obsts_prec_circles()
        self.publish_obsts_start_circles()

    def publish_obsts_markers(self):
        marker_array = MarkerArray()
        for i, obst in enumerate(self.model.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            marker.type = marker.CYLINDER
            marker.id = i+1
            # Set the scale of the marker
            marker.scale.x = obst.r
            marker.scale.y = obst.r
            marker.scale.z = 0.4
            # Set the color
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            # Set the pose of the marker
            marker.pose.position.x = obst.x
            marker.pose.position.y = obst.y
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
        for obst in self.model.obstacles:
            prec_circles = self.create_circle_points(obst.x, obst.y, obst.d_prec)
            obst_prec_points.extend(prec_circles)
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
        for obst in self.model.obstacles:
            prec_circles = self.create_circle_points(obst.x, obst.y, obst.d_start)
            obst_start_points.extend(prec_circles)
        obst_start_pc = PointCloud()
        obst_start_pc.header.frame_id = "map"
        obst_start_pc.points = obst_start_points
        self.publish_once(self.obst_start_pc_pub, obst_start_pc, "obst_start_pc ...")

    # ==================================================================================================================

    def publish_robots_prec_circles(self, xy_dict: dict):
        robot_prec_circles = []
        for xy in xy_dict.values():
            robot_circle = self.create_circle_points(xy[0], xy[1], self.params.robot_d_prec)
            robot_prec_circles.extend(robot_circle)
        robots_prec_pc = PointCloud()
        robots_prec_pc.header.frame_id = "map"
        robots_prec_pc.points = robot_prec_circles
        self.publish_once(self.robots_precs_pc_pub, robots_prec_pc, "robots_precs_circles ...")

    def publish_robots_start_circles(self, xy_dict: dict):
        robot_start_circles = []
        for xy in xy_dict.values():
            robot_circle = self.create_circle_points(xy[0], xy[1], self.params.robot_d_start)
            robot_start_circles.extend(robot_circle)
        robots_start_pc = PointCloud()
        robots_start_pc.header.frame_id = "map"
        robots_start_pc.points = robot_start_circles
        self.publish_normal(self.robots_starts_pc_pub, robots_start_pc, "robots_start_circles ...")

    def publish_robots_texts(self, xy_dict: dict):
        marker_array = MarkerArray()
        for rid, xy in xy_dict.items():
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
        self.publish_normal(self.robots_text_pub, marker_array, "vis: robots_texts ...")

    # ==================================================================================================================

    def visualize_arrow(self, x, y, theta, ns=None):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = marker.ARROW
        #
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

    def draw_fake_obsts_localmin(self, obstacles: List[Obstacle]):
        fake_obstacles = PointCloud()
        fake_obstacles.header.frame_id = "map"
        for obs in obstacles:
            circle = self.create_circle_points(obs.x, obs.y, obs.d_prec)
            fake_obstacles.points.extend(circle)
        self.fake_obsts_pc_pub.publish(fake_obstacles)

    def visualize_polygon(self, pols_xy: List[Tuple], ns: str, rid: int) -> None:
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
        pol_stamp.polygon = polygon
        self.robots_poly_pubs[ns].publish(pol_stamp)

    def draw_robot_circles(self, robots: List[ApfRobot], ns):
        """ Draw robots (ApfRobot) precaution circles
        """
        robot_circles = []
        for robot in robots:
            circle = self.create_circle_points(robot.x, robot.y, robot.r_prec)
            robot_circles.extend(circle)
        robot_prec_pc = PointCloud()
        robot_prec_pc.header.frame_id = "map"
        robot_prec_pc.points = robot_circles
        self.robots_circle_pubs[ns].publish(robot_prec_pc)

    # ==================================================================================================================

    def publish_normal(self, publisher, data, notif):
        publisher.publish(data)
        # rospy.loginfo(notif)
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

    def create_circle_points(self, c_x: float, c_y: float, radius: float) -> List[Point32]:
        circle_points = []
        for cos_th, sin_th in self.circle_points:
            p = Point32()
            p.x = c_x + radius * cos_th
            p.y = c_y + radius * sin_th
            circle_points.append(p)
        return circle_points
