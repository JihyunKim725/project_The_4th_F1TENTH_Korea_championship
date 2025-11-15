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

def _make_linestrip(frame_id, ns, mid, xs, ys, width, rgba, close_visually=True):
    """
    close_visually=True이면, 시각적으로 폐곡선이 되도록 첫 점을 맨 끝에 한 번 더 추가.
    (Marker 전용: 실제 주행 경로 배열에는 중복점 추가하지 않음)
    """
    m = Marker()
    m.header.frame_id = frame_id; m.header.stamp = rospy.Time.now()
    m.ns = ns; m.id = mid
    m.type = Marker.LINE_STRIP; m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(width)
    m.color.r, m.color.g, m.color.b, m.color.a = rgba

    pts_x = xs; pts_y = ys
    if close_visually and len(xs) > 1:
        if xs[0] != xs[-1] or ys[0] != ys[-1]:
            pts_x = np.concatenate([xs, [xs[0]]])
            pts_y = np.concatenate([ys, [ys[0]]])

    m.points = [Point(x=float(x), y=float(y), z=0.0) for x,y in zip(pts_x, pts_y)]
    return m

def _roll(array_like, start_idx):
    """start_idx부터 시작하도록 순환(roll)된 새 배열 반환"""
    a = np.asarray(array_like)
    if start_idx <= 0: return a.copy()
    return np.concatenate([a[start_idx:], a[:start_idx]])

def _build_cyclic_path(p, start_idx):
    """
    경로 p(dict: x,y,psi,kappa,vx,ax)를 start_idx부터 시작하는 '순환 배열'로 생성.
    (단일 경로 루프)  -> 항상 폐곡선 성질을 갖도록 함.
    """
    out = {}
    for k in ["x","y","psi","kappa","vx","ax"]:
        out[k] = _roll(p[k], start_idx)
    out["s"] = _compute_s(out["x"], out["y"])
    return out

# ---------- main ----------
class LaneVisualize:
    def __init__(self):
        rospy.init_node("lane_visualize_node")

        # Files (네가 쓰는 기본값 유지)
        self.map_name    = rospy.get_param("~map_name", "0924")
        self.traj_file_1 = rospy.get_param("~traj_file_1", "traj_race_cl_1")
        self.traj_file_2 = rospy.get_param("~traj_file_2", "traj_race_cl_2")

        # CSV load
        traj_csv_loc_1 = os.path.join("/home/jihyun/maps", self.map_name, self.traj_file_1 + ".csv")
        traj_csv_loc_2 = os.path.join("/home/jihyun/maps", self.map_name, self.traj_file_2 + ".csv")
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

        # 겹침 사전 계산(1→2, 2→1)
        self.over12 = _find_overlap_indices(self.p[0]["x"], self.p[0]["y"], self.p[0]["psi"],
                                            self.p[1]["x"], self.p[1]["y"], self.p[1]["psi"],
                                            d_thresh=0.20, psi_thresh=0.26)
        self.over21 = _find_overlap_indices(self.p[1]["x"], self.p[1]["y"], self.p[1]["psi"],
                                            self.p[0]["x"], self.p[0]["y"], self.p[0]["psi"],
                                            d_thresh=0.20, psi_thresh=0.26)

        # 초기 퍼블리시: path1을 '순환 배열'로
        self.active_path = 1  # 현재 선택된 경로 (1 or 2)
        start_idx = 0
        if self.robot_xy is not None:
            start_idx = _nearest_index_to_point(self.robot_xy[0], self.robot_xy[1],
                                                self.p[0]["x"], self.p[0]["y"])[0]
        self.final = _build_cyclic_path(self.p[0], start_idx)
        self._publish_all()

        # keep alive
        rospy.Timer(rospy.Duration(0.5), lambda _ : self._publish_all())

    # ---------- Callbacks ----------
    def _pose_cb(self, msg):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _switch_cb(self, msg):
        target = int(msg.data)  # 1 or 2
        if target not in (1,2): return
        if target == self.active_path:
            rospy.loginfo_throttle(1, f"[lane_visualize] already on traj{target}")
            return

        # 겹침 정보 선택 (from -> to)
        if self.active_path == 1 and target == 2 and self.over12 is not None:
            i0, i1, j_map = self.over12
            from_idx = 0; to_idx = 1
            to_map = j_map;  base_from = self.p[from_idx]
        elif self.active_path == 2 and target == 1 and self.over21 is not None:
            i0, i1, j_map = self.over21
            from_idx = 1; to_idx = 0
            to_map = j_map;  base_from = self.p[from_idx]
        else:
            rospy.logwarn("[lane_visualize] No overlap for requested switch; keeping current path.")
            return

        # 겹침 구간 내에서 '현재 위치'에 가장 가까운 from-경로 인덱스(i_sw) 선택
        if self.robot_xy is None:
            ii = np.arange(i0, i1+1)
            i_sw = int(ii[len(ii)//2])  # fallback: 겹침 중앙
        else:
            ii = np.arange(i0, i1+1)
            xs = base_from["x"][ii]; ys = base_from["y"][ii]
            k = _nearest_index_to_point(self.robot_xy[0], self.robot_xy[1], xs, ys)[0]
            i_sw = int(ii[k])

        # 대응하는 to-경로 인덱스
        j_sw = int(to_map[i_sw - i0])

        # 🔁 핵심 변경: 전환 후에는 'to 경로 전체'를 j_sw부터 시작하는 순환 배열로 퍼블리시
        self.final = _build_cyclic_path(self.p[to_idx], j_sw)
        self.active_path = target
        rospy.loginfo(f"[lane_visualize] SWITCH traj{from_idx+1}[{i_sw}]  ->  traj{to_idx+1}[{j_sw}] (cyclic)")

        # 즉시 퍼블리시
        self._publish_all()

    # ---------- Publish ----------
    def _publish_all(self):
        # 원본 두 경로(파랑/빨강) - 시각적 폐곡선 보정 on
        m1 = _make_linestrip("map","global_planner",1, self.p[0]["x"], self.p[0]["y"], 0.04, (0.0,0.6,1.0,1.0), close_visually=True)
        m2 = _make_linestrip("map","global_planner",2, self.p[1]["x"], self.p[1]["y"], 0.04, (1.0,0.3,0.3,1.0), close_visually=True)
        # 최종 경로(초록, 순환 배열) - 시각적 폐곡선 보정 on
        mf = _make_linestrip("map","global_planner",10, self.final["x"], self.final["y"], 0.04, (0.1,1.0,0.4,1.0), close_visually=True)
        self.mk_p1.publish(m1); self.mk_p2.publish(m2); self.mk_final.publish(mf)

        # waypoints (중복점 없이 순환 배열만 퍼블리시)
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
