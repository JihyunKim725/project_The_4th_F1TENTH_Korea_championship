#!/usr/bin/env python3
import time
import sys
from typing import List, Tuple

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline

from f110_msgs.msg import ObstacleArray, OTWpntArray, Wpnt, WpntArray #, FrenetState
from frenet_utils import FrenetConverter

class StaticObstacleSpliner:
    """
    A ROS node for generating spline paths around static obstacles.
    """

    def __init__(self):
        rospy.init_node("static_obs_spliner_node")

        # Variables
        self.obs = ObstacleArray()
        self.gb_wpnts = WpntArray()
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = 0
        self.cur_d = 0
        self.cur_vs = 0
        self.gb_scaled_wpnts = WpntArray()
        # self.lookahead = 1  # in meters [m]

        # Parameters
        self.pre_apex_0 = -4
        self.pre_apex_1 = -3
        self.pre_apex_2 = -1.5
        self.post_apex_0 = 1
        self.post_apex_1 = 2
        self.post_apex_2 = 3
        self.evasion_dist = 0.4
        self.obs_traj_tresh = 0.6
        self.spline_bound_mindist = 0.5

        self.initial_wpnts = None
        self.initial_mrks = None 

        # Subscribers
        rospy.Subscriber("/global_path/optimal_trajectory_wpnt", WpntArray, self.gb_cb)
        
        # Initialize Frenet Converter
        self.converter = self.initialize_converter()

        rospy.Subscriber("/perception/detection/raw_obstacles", ObstacleArray, self.obs_cb)
        rospy.Subscriber("/odom_frenet", Odometry, self.state_cb)


        # Publishers
        self.mrks_pub = rospy.Publisher("/planner/avoidance/markers", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher("/planner/avoidance/otwpnts", OTWpntArray, queue_size=10)

        # Initialize Frenet Converter
        self.converter = self.initialize_converter()

        # Loop rate
        self.rate = rospy.Rate(20)  # Hz

    # Callbacks
    def obs_cb(self, data: ObstacleArray):
        self.obs = data

    def state_cb(self, data: Odometry):
        self.cur_vs = data.twist.twist.linear.x
        self.cur_s, self.cur_d = data.pose.pose.position.x ,data.pose.pose.position.y
        


    def gb_cb(self, data: WpntArray):
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        self.gb_wpnts = data
        if self.gb_vmax is None:
            self.gb_vmax = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
            self.gb_max_idx = data.wpnts[-1].id
            self.gb_max_s = data.wpnts[-1].s_m

    # Main loop
    def loop(self):
        rospy.loginfo("[static_obs_spliner_node] Waiting for messages...")
        rospy.wait_for_message("/global_path/optimal_trajectory_wpnt", WpntArray)
        rospy.wait_for_message("/perception/detection/raw_obstacles", ObstacleArray)
        rospy.loginfo("[static_obs_spliner_node] Ready!")
    
        while not rospy.is_shutdown():
            obs = self.obs
            gb_wpnts = self.gb_wpnts.wpnts

            # obs.obstacles가 아닌, 필터링된 close_obs를 기준으로 판단
            close_obs = [o for o in obs.obstacles if abs(o.d_center) < self.obs_traj_tresh]

            wpnts, mrks = OTWpntArray(), MarkerArray()

            if len(obs.obstacles) > 0:
                wpnts, mrks = self.do_spline(obstacles=obs, gb_wpnts=gb_wpnts)
            else:
                # 회피할 장애물이 없으면 글로벌 경로를 그대로 발행
                # (이전에 발행된 스플라인 마커는 삭제)
                wpnts.wpnts = self.gb_wpnts.wpnts 
                wpnts.header.stamp = rospy.Time.now()
                wpnts.header.frame_id = "map"
                
                del_mrk = Marker()
                del_mrk.action = Marker.DELETEALL
                mrks.markers.append(del_mrk)

            self.evasion_pub.publish(wpnts)
            self.mrks_pub.publish(mrks)
            self.rate.sleep()

    # def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[OTWpntArray, MarkerArray]:
    #     """
    #     Creates a spline path around static obstacles.
    #     """
    #     wpnts = OTWpntArray()
    #     mrks = MarkerArray()

    #     # Filter obstacles near the path
    #     close_obs = [obs for obs in obstacles.obstacles if abs(obs.d_center) < self.obs_traj_tresh]

    #     if len(close_obs) > 0:
    #         closest_obs = min(close_obs, key=lambda obs: (obs.s_center - self.cur_s) % self.gb_max_s)

    #         s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
    #         if closest_obs.d_center < 0: 
    #             d_apex = closest_obs.d_center + self.evasion_dist
    #         else: 
    #             d_apex = -(closest_obs.d_center + self.evasion_dist)
    #         evasion_points = []
    #         spline_params = [
    #             self.pre_apex_0, self.pre_apex_1, self.pre_apex_2,
    #             0,
    #             self.post_apex_0, self.post_apex_1, self.post_apex_2
    #         ]

    #         for dst in spline_params:
    #             si = s_apex + dst
    #             di = d_apex if dst == 0 else 0
    #             evasion_points.append([si, di])

    #         evasion_points = np.array(evasion_points)

    #         spatial_spline = Spline(x=evasion_points[:, 0], y=evasion_points[:, 1])
    #         evasion_s = np.arange(evasion_points[0, 0], evasion_points[-1, 0], 0.1)
    #         evasion_d = spatial_spline(evasion_s)

    #         evasion_s = evasion_s % self.gb_max_s
    #         evasion_d = np.clip(evasion_d, -self.evasion_dist, self.evasion_dist)

    #         resp = self.converter.get_cartesian(evasion_s, evasion_d)
    #         line_strip_points = []

    #         for i in range(evasion_s.shape[0]):
    #             gb_wpnt_i = int((evasion_s[i] / (self.gb_wpnts.wpnts[1].s_m - self.gb_wpnts.wpnts[0].s_m)) % self.gb_max_idx)
    #             # Calculate the speed for the waypoint based on its position relative to apex
    #             if evasion_s[i] < s_apex + self.pre_apex_2:  # Pre-apex: 감속 구간
    #                 vi =  self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 0.5 # 현재 속도의 60%로 감속, 최소 0.5 m/s 유지
    #             elif s_apex + self.post_apex_0 <= evasion_s[i] <= s_apex + self.post_apex_2:  # Post-apex: 가속 구간
    #                 vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 0.7 # 최대 속도를 넘지 않으며 현재 속도의 120%로 가속
    #             else:  # 코너 외부: 기존 글로벌 속도 사용
    #                 vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 0.5

    #             # Add waypoints to OTWpntArray message
    #             wpnts.wpnts.append(Wpnt(
    #                 id=i,
    #                 x_m=resp[0, i],
    #                 y_m=resp[1, i],
    #                 s_m=evasion_s[i],
    #                 d_m=evasion_d[i],
    #                 vx_mps=vi
    #             ))

    #             # Collect points for the line strip marker
    #             line_strip_points.append([resp[0, i], resp[1, i]])

    #         # Create a single LINE_STRIP marker for all points
    #         marker_id = 0  # Single ID for the LINE_STRIP marker
    #         line_strip_marker = self.create_marker(line_strip_points, marker_id)
    #         mrks.markers.append(line_strip_marker)


    #         wpnts.header.stamp = rospy.Time.now()
    #         wpnts.header.frame_id = "map"

    #     return wpnts, mrks
    def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[OTWpntArray, MarkerArray]:
        """
        Creates a spline path around static obstacles, dividing d_center into three ranges.
        """
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # Filter obstacles near the path
        close_obs = [obs for obs in obstacles.obstacles if abs(obs.d_center) < self.obs_traj_tresh]

        # Define range thresholds
        threshold_low = 0.2
        threshold_high = 0.5

        if len(close_obs) > 0:
            closest_obs = min(close_obs, key=lambda obs: (obs.s_center - self.cur_s) % self.gb_max_s)

            s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
            d_center = closest_obs.d_center

            # Determine range
            if abs(d_center) < threshold_low:
                # Low range: Use a narrow evasion distance
                d_apex = d_center + self.evasion_dist
            elif threshold_low <= abs(d_center) <= threshold_high:
                # Mid range: Choose either low or high based on a condition
                if d_center > 0:
                    d_apex = -(threshold_high + self.evasion_dist)  # Example: Select high range on the left
                else:
                    d_apex = threshold_low + self.evasion_dist  # Example: Select low range on the right
            else:
                # High range: Use a wide evasion distance
                d_apex = d_center + 2 * self.evasion_dist if d_center < 0 else -(d_center + 2 * self.evasion_dist)

            # Define spline parameters
            evasion_points = []
            spline_params = [
                self.pre_apex_0, self.pre_apex_1, self.pre_apex_2,
                0,
                self.post_apex_0, self.post_apex_1, self.post_apex_2
            ]

            for dst in spline_params:
                si = s_apex + dst
                di = d_apex if dst == 0 else 0
                evasion_points.append([si, di])

            evasion_points = np.array(evasion_points)

            spatial_spline = Spline(x=evasion_points[:, 0], y=evasion_points[:, 1])
            evasion_s = np.arange(evasion_points[0, 0], evasion_points[-1, 0], 0.1)
            evasion_d = spatial_spline(evasion_s)

            evasion_s = evasion_s % self.gb_max_s
            evasion_d = np.clip(evasion_d, -self.evasion_dist, self.evasion_dist)

            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            line_strip_points = []

            for i in range(evasion_s.shape[0]):
                gb_wpnt_i = int((evasion_s[i] / (self.gb_wpnts.wpnts[1].s_m - self.gb_wpnts.wpnts[0].s_m)) % self.gb_max_idx)

                # Calculate the speed for the waypoint based on its position relative to apex
                if evasion_s[i] < s_apex + self.pre_apex_2:
                    vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 0.9
                elif s_apex + self.post_apex_0 <= evasion_s[i] <= s_apex + self.post_apex_2:
                    vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 1.0
                else:
                    vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps * 0.9

                wpnts.wpnts.append(Wpnt(
                    id=i,
                    x_m=resp[0, i],
                    y_m=resp[1, i],
                    s_m=evasion_s[i],
                    d_m=evasion_d[i],
                    vx_mps=vi
                ))

                line_strip_points.append([resp[0, i], resp[1, i]])

            marker_id = 0
            line_strip_marker = self.create_marker(line_strip_points, marker_id)
            mrks.markers.append(line_strip_marker)

            wpnts.header.stamp = rospy.Time.now()
            wpnts.header.frame_id = "map"

        return wpnts, mrks

 
    def create_marker(self, waypoints: list, marker_id: int) -> Marker:
        """
        Creates a LINE_STRIP marker for RViz visualization.
        
        Args:
            waypoints (list): A list of waypoints, where each waypoint is a [x, y] pair.
            marker_id (int): Unique ID for the marker.
        
        Returns:
            Marker: Configured LINE_STRIP marker.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Thickness of the line
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Add points to the marker
        for waypoint in waypoints:
            point = Point()
            point.x = waypoint[0]  # X-coordinate
            point.y = waypoint[1]  # Y-coordinate
            point.z = 0.0  # Assume Z=0 for 2D visualization
            marker.points.append(point)

        # Assign a unique ID for the marker
        marker.id = marker_id

        return marker



    def initialize_converter(self) -> FrenetConverter:
        rospy.wait_for_message("/global_path/optimal_trajectory_wpnt", WpntArray)
        converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1], None)
        rospy.loginfo("[static_obs_spliner_node] FrenetConverter initialized.")
        return converter

if __name__ == "__main__":
    spliner = StaticObstacleSpliner()
    spliner.loop()
