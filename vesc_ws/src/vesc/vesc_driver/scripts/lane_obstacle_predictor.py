#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, numpy as np
import rospy, tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from f110_msgs.msg import WpntArray, OTWpntArray, OTWpnt

def _wrap(a): return math.atan2(math.sin(a), math.cos(a))

def _nearest_idx_xy(px, py, xs, ys):
    d = np.hypot(xs - px, ys - py)
    i = int(np.argmin(d))
    return i, float(d[i])

def _avg_ds(x, y):
    if len(x) < 2: return 0.1
    return float(np.mean(np.hypot(np.diff(x), np.diff(y))))

def _indices_by_distance(i_center, s_window_m, ds_avg, n_total):
    half = max(1, int(max(s_window_m, 0.5)/max(ds_avg,1e-3)/2))
    i0 = max(0, i_center - half)
    i1 = min(n_total-1, i_center + half)
    return i0, i1

def _raised_cosine_profile(n, peak):
    # 0 -> peak -> 0, 길이 n
    t = np.linspace(0, np.pi, max(2, n))
    return peak * 0.5*(1 - np.cos(t))

def _yaw_from_quat(q):
    import tf.transformations as tft
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

class LaneObstaclePredictor:
    def __init__(self):
        rospy.init_node("lane_obstacle_predictor")
        self.frame_map = rospy.get_param("~map_frame", "map")
        self.frame_base = rospy.get_param("~base_frame", "base_link")

        # 고속 주행 기본 파라미터(제트슨 Orin Nano에서도 가볍게 돌도록)
        self.front_sector_deg = rospy.get_param("~front_sector_deg", 30.0)   # 전방 ±각도
        self.obs_dist_thresh  = rospy.get_param("~obs_dist_thresh", 3.0)     # 이 거리 안이면 회피 시작
        self.left_sector_deg  = rospy.get_param("~left_sector_deg", 60.0)    # 좌/우 여유 판단에 쓸 섹터
        self.right_sector_deg = rospy.get_param("~right_sector_deg", 60.0)
        self.lateral_offset_max = rospy.get_param("~lateral_offset_max", 0.35)  # m (트랙폭/차폭 고려)
        self.window_len_m = rospy.get_param("~window_len_m", 10.0)           # 회피 적용 전체 길이
        self.subset_len_m = rospy.get_param("~subset_len_m", 30.0)           # 퍼블리시할 부분 경로 길이
        self.ramp_ratio   = rospy.get_param("~ramp_ratio", 0.3)              # 앞/뒤 램프 비율(부드러운 진입/이탈)

        # 상태
        self.has_scan = False
        self.pose_xyyaw = None  # (x, y, yaw)
        self.path = None        # dict: x,y,psi,s,vx
        self.ds_avg = 0.1

        # Pub/Sub
        self.pub_ot = rospy.Publisher("/planner/avoidance/otwpnts", OTWpntArray, queue_size=1)
        rospy.Subscriber("/global_path/optimal_trajectory_wpnt", WpntArray, self.cb_path, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.cb_pose, queue_size=20)
        rospy.Subscriber("/scan", LaserScan, self.cb_scan, queue_size=1)

        rospy.loginfo("[lane_obstacle_predictor] ready.")

    # ---- Callbacks ----
    def cb_path(self, msg: WpntArray):
        n = len(msg.wpnts)
        x = np.empty(n); y = np.empty(n); psi = np.empty(n); s = np.empty(n); vx = np.empty(n)
        for i, w in enumerate(msg.wpnts):
            x[i] = w.x_m; y[i] = w.y_m
            psi[i] = w.psi_rad if hasattr(w, "psi_rad") else 0.0
            s[i]   = w.s_m     if hasattr(w, "s_m")     else (s[i-1]+0.1 if i>0 else 0.0)
            vx[i]  = w.vx_mps  if hasattr(w, "vx_mps")  else 2.0
        self.path = {"x":x, "y":y, "psi":psi, "s":s, "vx":vx}
        self.ds_avg = _avg_ds(x, y)

    def cb_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q)
        self.pose_xyyaw = (p.x, p.y, yaw)

    def cb_scan(self, scan: LaserScan):
        if self.path is None or self.pose_xyyaw is None:
            # 경로/자세 없으면 회피 publish 비움
            self.pub_ot.publish(OTWpntArray()) 
            return

        # 1) 전방 섹터 필터링
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment  # base_link 기준
        ranges = np.array(scan.ranges, dtype=float)
        # 유효값만
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        # 전방 ±front_sector_deg
        sector_rad = math.radians(self.front_sector_deg)
        front_mask = valid & (angles > -sector_rad) & (angles < sector_rad)
        if not np.any(front_mask):
            self.pub_ot.publish(OTWpntArray())
            return

        r_front = ranges[front_mask]
        a_front = angles[front_mask]
        rmin = float(np.min(r_front))
        amin = float(a_front[np.argmin(r_front)])

        # 장애물 판정
        if rmin > self.obs_dist_thresh:
            # 장애물 없음 → 비움
            self.pub_ot.publish(OTWpntArray())
            return

        # 2) 좌/우 여유 측정 (median 거리 큰 쪽 선택)
        left_mask  = valid & (angles > math.radians(10.0))  & (angles < math.radians(10.0 + self.left_sector_deg))
        right_mask = valid & (angles < math.radians(-10.0)) & (angles > math.radians(-10.0 - self.right_sector_deg))
        left_med  = float(np.median(ranges[left_mask]))  if np.any(left_mask)  else 0.0
        right_med = float(np.median(ranges[right_mask])) if np.any(right_mask) else 0.0
        side = +1.0 if left_med >= right_med else -1.0  # +1: 좌로 비킴, -1: 우로 비킴

        # 3) 충돌점(base_link) → map 변환
        # base_link에서 전방 rmin, 각 amin의 점
        ox_bl = rmin * math.cos(amin)
        oy_bl = rmin * math.sin(amin)
        x, y, yaw = self.pose_xyyaw
        # 회전 후 평행이동
        c, s = math.cos(yaw), math.sin(yaw)
        ox_map = x + c*ox_bl - s*oy_bl
        oy_map = y + s*ox_bl + c*oy_bl

        # 4) 글로벌 경로 최근접 인덱스
        i_obs, d_obs = _nearest_idx_xy(ox_map, oy_map, self.path["x"], self.path["y"])

        # 5) 회피 구간 인덱스 범위 산정
        # - ramp_ratio 비율로 앞/뒤 램프, 가운데 plateau
        N_total = len(self.path["x"])
        i0, i1 = _indices_by_distance(i_obs, self.window_len_m, self.ds_avg, N_total)

        # 6) raised-cosine 오프셋 프로파일 (0→peak→0)
        n_win = max(3, i1 - i0 + 1)
        prof = _raised_cosine_profile(n_win, self.lateral_offset_max) * side  # 좌/우 부호 반영

        # 7) 회피 부분 경로 생성 (법선 방향으로 offset)
        seg_x = self.path["x"][i0:i1+1].copy()
        seg_y = self.path["y"][i0:i1+1].copy()
        seg_psi = self.path["psi"][i0:i1+1].copy()
        nx = -np.sin(seg_psi); ny = np.cos(seg_psi)
        seg_x += prof * nx
        seg_y += prof * ny

        # 8) 퍼블리시할 subset 길이(앞쪽으로 subset_len_m) 확보
        #    i_obs부터 앞쪽으로 subset_len_m 커버되도록 끝 인덱스 확장
        i_sub0 = i0
        remain_m = self.subset_len_m
        i_sub1 = i0
        while i_sub1 < N_total-1 and remain_m > 0:
            step = math.hypot(self.path["x"][i_sub1+1]-self.path["x"][i_sub1],
                              self.path["y"][i_sub1+1]-self.path["y"][i_sub1])
            remain_m -= step
            i_sub1 += 1

        # subset을 구성: [i0..i1]은 offset 적용, 나머지는 원본
        out_x = self.path["x"][i_sub0:i_sub1+1].copy()
        out_y = self.path["y"][i_sub0:i_sub1+1].copy()
        out_v = self.path["vx"][i_sub0:i_sub1+1].copy()

        # offset 적용 부분 덮어쓰기
        j0 = 0                           # out에서 i0에 해당하는 로컬 시작
        j1 = j0 + (i1 - i0)              # out에서 i1에 해당하는 로컬 끝
        out_x[j0:j1+1] = seg_x
        out_y[j0:j1+1] = seg_y
        # 속도는 그대로(감속 최소화). 필요하면 상한만 두고 clip 가능:
        # out_v = np.clip(out_v, 0.0, vmax_clip)

        # 9) 메시지 퍼블리시 (OTWpntArray)
        msg = OTWpntArray()
        for xi, yi, vi in zip(out_x, out_y, out_v):
            w = OTWpnt()
            # 필드 명은 f110_msgs에 맞춰 x_m, y_m만 채워도 MAP_controller가 속도 2.0으로 세팅함
            w.x_m = float(xi); w.y_m = float(yi)
            # 필요하면 속도 필드가 있다면 채워도 됨 (패키지 정의에 따라)
            msg.wpnts.append(w)
        self.pub_ot.publish(msg)
        rospy.loginfo_throttle(1.0, f"[avoid] obs={rmin:.2f}m @i={i_obs}  side={'L' if side>0 else 'R'}  "
                                   f"win=[{i0},{i1}] out=[{i_sub0},{i_sub1}] n={len(msg.wpnts)}")

if __name__ == "__main__":
    LaneObstaclePredictor()
    rospy.spin()
