#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os, math
import numpy as np
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32
from f110_msgs.msg import WpntArray, Wpnt

# ---------- helpers ----------
def _wrap_angle(a): return math.atan2(math.sin(a), math.cos(a))
def _heading_diff(a, b): return abs(_wrap_angle(a - b))

def _compute_s(x, y):
    if len(x) == 0: return np.array([], dtype=float)
    ds = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate([[0.0], np.cumsum(ds)])

def _nearest_index_to_point(px, py, xs, ys):
    d = np.hypot(xs - px, ys - py)
    return int(np.argmin(d)), float(np.min(d))

def _find_overlap_indices(x1, y1, psi1, x2, y2, psi2, d_thresh=0.20, psi_thresh=0.26):
    N1, N2 = len(x1), len(x2)
    if N1==0 or N2==0: return None
    dx = x1[:,None]-x2[None,:]; dy = y1[:,None]-y2[None,:]
    dist = np.hypot(dx, dy)
    j_min = np.argmin(dist, axis=1)
    d_min = dist[np.arange(N1), j_min]
    psi_diff = np.array([_heading_diff(psi1[i], psi2[j_min[i]]) for i in range(N1)])
    mask = (d_min < d_thresh) & (psi_diff < psi_thresh)
    if not np.any(mask): return None
    idx = np.where(mask)[0]
    ranges = []; s = idx[0]
    for k in range(1,len(idx)):
        if idx[k] != idx[k-1]+1:
            ranges.append((s, idx[k-1])); s = idx[k]
    ranges.append((s, idx[-1]))
    i0, i1 = max(ranges, key=lambda t: (t[1]-t[0]))
    return int(i0), int(i1), j_min[i0:i1+1]

def _make_linestrip(frame_id, ns, mid, xs, ys, width, rgba):
    m = Marker()
    m.header.frame_id = frame_id; m.header.stamp = rospy.Time.now()
    m.ns = ns; m.id = mid
    m.type = Marker.LINE_STRIP; m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(width)
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.points = [Point(x=float(x), y=float(y), z=0.0) for x,y in zip(xs,ys)]
    return m

# ---------- main ----------
class LaneVisualize:
    def __init__(self):
        rospy.init_node("lane_visualize_node")

        # Files (기본값은 현재 쓰는 것과 동일)
        self.map_name    = rospy.get_param("~map_name", "0924")
        self.traj_file_1 = rospy.get_param("~traj_file_1", "traj_race_cl_1")
        self.traj_file_2 = rospy.get_param("~traj_file_2", "traj_race_cl_2")

        # CSV load
        traj_csv_loc_1 = os.path.join("/home/nvidia/maps", self.map_name, self.traj_file_1 + ".csv")
        traj_csv_loc_2 = os.path.join("/home/nvidia/maps", self.map_name, self.traj_file_2 + ".csv")
        traj_data_1 = np.loadtxt(traj_csv_loc_1, delimiter=";", skiprows=1)
        traj_data_2 = np.loadtxt(traj_csv_loc_2, delimiter=";", skiprows=1)

        self.p = [
            { # path 1
                "s": traj_data_1[:,0], "x": traj_data_1[:,1], "y": traj_data_1[:,2],
                "psi": traj_data_1[:,3], "kappa": traj_data_1[:,4],
                "vx": traj_data_1[:,5], "ax": traj_data_1[:,6],
            },
            { # path 2
                "s": traj_data_2[:,0], "x": traj_data_2[:,1], "y": traj_data_2[:,2],
                "psi": traj_data_2[:,3], "kappa": traj_data_2[:,4],
                "vx": traj_data_2[:,5], "ax": traj_data_2[:,6],
            },
        ]

        # Pubs
        self.wp_pub     = rospy.Publisher("/global_path/optimal_trajectory_wpnt", WpntArray, queue_size=10, latch=True)
        self.mk_final   = rospy.Publisher("/global_path/optimal_trajectory_marker", Marker, queue_size=10, latch=True)
        self.mk_p1      = rospy.Publisher("/global_path/path1_marker", Marker, queue_size=10, latch=True)
        self.mk_p2      = rospy.Publisher("/global_path/path2_marker", Marker, queue_size=10, latch=True)

        # Subs: 현재 위치 + 전환 명령
        self.robot_xy = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber("/global_path/switch_to", Int32, self._switch_cb, queue_size=10)

        # 겹침 사전 계산(1→2, 2→1 모두)
        self.over12 = _find_overlap_indices(self.p[0]["x"], self.p[0]["y"], self.p[0]["psi"],
                                            self.p[1]["x"], self.p[1]["y"], self.p[1]["psi"],
                                            d_thresh=0.20, psi_thresh=0.26)
        self.over21 = _find_overlap_indices(self.p[1]["x"], self.p[1]["y"], self.p[1]["psi"],
                                            self.p[0]["x"], self.p[0]["y"], self.p[0]["psi"],
                                            d_thresh=0.20, psi_thresh=0.26)

        # 시작은 1번 경로를 전체로(필요 시 파라미터로 바꿔도 됨)
        self.active_from = 1
        self.final = self._assemble_full(0)  # path1 full
        self._publish_all()

        # keep alive
        rospy.Timer(rospy.Duration(0.5), lambda _ : self._publish_all())

    # ---------- Callbacks ----------
    def _pose_cb(self, msg):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _switch_cb(self, msg):
        # msg.data: 1 또는 2 (목표 경로)
        target = int(msg.data)
        if target not in (1,2): return
        src = self.active_from
        if src == target:
            rospy.loginfo_throttle(1, f"[lane_visualize] already on traj{target}")
            return

        # 겹침 정보 선택
        if src == 1 and target == 2 and self.over12 is not None:
            i0, i1, j_map = self.over12
            from_idx = 0; to_idx = 1
        elif src == 2 and target == 1 and self.over21 is not None:
            i0, i1, j_map = self.over21
            from_idx = 1; to_idx = 0
        else:
            rospy.logwarn("[lane_visualize] No overlap for requested switch; keeping current path.")
            return

        # 현재 위치 근처의 from-경로 인덱스 선택 (겹침 구간 안에서)
        if self.robot_xy is None:
            # 포즈가 없으면 겹침 중앙으로 fallback
            ii = np.arange(i0, i1+1)
            i_sw = int(ii[len(ii)//2])
        else:
            ii = np.arange(i0, i1+1)
            xs = self.p[from_idx]["x"][ii]; ys = self.p[from_idx]["y"][ii]
            k = _nearest_index_to_point(self.robot_xy[0], self.robot_xy[1], xs, ys)[0]
            i_sw = int(ii[k])

        # 対응하는 to-경로 인덱스
        j_sw = int(j_map[i_sw - i0])

        # 블렌딩 없이 바로 전환: from[:i_sw] + to[j_sw:]
        self.final = self._assemble_switch(from_idx, to_idx, i_sw, j_sw)
        self.active_from = target
        rospy.loginfo(f"[lane_visualize] switch traj{src}({i_sw}) -> traj{target}({j_sw})")

        # 즉시 퍼블리시
        self._publish_all()

    # ---------- Core ----------
    def _assemble_full(self, idx):
        out = {}
        for k in ["x","y","psi","kappa","vx","ax"]:
            out[k] = self.p[idx][k].copy()
        out["s"] = _compute_s(out["x"], out["y"])
        return out

    def _assemble_switch(self, from_idx, to_idx, i_sw, j_sw):
        out = {}
        for k in ["x","y","psi","kappa","vx","ax"]:
            out[k] = np.concatenate([ self.p[from_idx][k][:i_sw+1],
                                      self.p[to_idx][k][j_sw:] ])
        out["s"] = _compute_s(out["x"], out["y"])
        return out

    # ---------- Publish ----------
    def _publish_all(self):
        # markers
        m1 = _make_linestrip("map","global_planner",1, self.p[0]["x"], self.p[0]["y"], 0.04, (0.0,0.6,1.0,1.0))
        m2 = _make_linestrip("map","global_planner",2, self.p[1]["x"], self.p[1]["y"], 0.04, (1.0,0.3,0.3,1.0))
        mf = _make_linestrip("map","global_planner",10, self.final["x"], self.final["y"], 0.04, (0.1,1.0,0.4,1.0))
        self.mk_p1.publish(m1); self.mk_p2.publish(m2); self.mk_final.publish(mf)

        # waypoints
        wpa = WpntArray()
        n = len(self.final["x"])
        for i in range(n):
            w = Wpnt()
            w.id = i
            w.s_m = float(self.final["s"][i])
            w.x_m = float(self.final["x"][i]); w.y_m = float(self.final["y"][i])
            w.psi_rad = float(self.final["psi"][i]); w.kappa_radpm = float(self.final["kappa"][i])
            w.vx_mps = float(self.final["vx"][i]); w.ax_mps2 = float(self.final["ax"][i])
            wpa.wpnts.append(w)
        self.wp_pub.publish(wpa)

if __name__ == "__main__":
    print("Lane Visualize Initialized")
    LaneVisualize()
    rospy.spin()

