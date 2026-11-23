"""SLAM 맵핑 중 ArUco 마커 위치를 지도(map) 좌표로 기록하고, 웨이포인트를 자동으로 만든다 (Orin에서 실행)

slam_toolbox 가 지도를 그리는 동안 라파의 /aruco/* 를 받아 각 마커의 지도 좌표를 모은다.
저장하면 두 파일이 생긴다.
  ~/slam_nav2/maps/<map_name>_markers.yaml   마커 지도 좌표 → Nav2 주행 중 aruco_localizer 가 위치 보정에 사용
  ~/slam_nav2/waypoints.yaml                 START + 마커마다 하나 (P1, P2, TRASH=거치대 ID 3, BIN=쓰레기통 ID 0), 발견 순서
  aruco_explore.py 와 함께 쓰면 각 마커(= 맵의 끝) 앞에 정렬해 멈춘 위치의 값이 기록된다.

  python3 marker_mapper.py                                  # 반드시 '출발점'에서 켠다 (켤 때 위치 = START)
  ros2 service call /marker_mapper/set_start std_srvs/srv/Trigger   # 지금 위치를 START 로 다시 지정
  ros2 service call /marker_mapper/status std_srvs/srv/Trigger      # 기록 현황
  ros2 service call /marker_mapper/save std_srvs/srv/Trigger        # 저장 (Ctrl+C 로 끌 때도 자동 저장)

정확도를 위해 지키는 것
  - 로봇이 거의 멈춰 있을 때(|v|<0.05 m/s, |w|<0.15 rad/s)만 기록한다 → 마커 앞에서 2~3초 멈출 것
  - 마커와 0.2~1.5 m, 정면에서 45° 이내일 때만 기록한다
  - 마커마다 '가장 가까이에서 본 방문'(정렬해서 멈춘 때)의 측정값을 쓰고, 비슷하게 가까운 방문이 여럿이면
    가장 최근 것을 쓴다 (SLAM 루프 클로저로 지도가 보정된 뒤의 값)
"""
import math
import os
import shutil
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)

from marker_geom import approach_point, median_pose, quat_z_axis, yaw_of

SLAM_DIR = os.path.expanduser(os.environ.get("SLAM_NAV2", "~/slam_nav2"))

# 마커 이름은 aruco_node.py 의 MARKERS 와 같아야 한다.
#   wp: 웨이포인트 이름, dist: 웨이포인트 = 마커 표면 법선 방향 dist(m) 앞 (base_link 원점 기준)
MARKERS = {
    1: dict(name="wall_1", wp="P1", dist=0.60),
    2: dict(name="wall_2", wp="P2", dist=0.60),
    3: dict(name="stand", wp="TRASH", dist=0.55),                     # 캔 거치대
    0: dict(name="bin", wp="BIN", dist=0.55, radius=0.095),           # 원형 쓰레기통
}


class MarkerMapper(Node):
    def __init__(self):
        super().__init__("marker_mapper")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.map_frame = p("map_frame", "map")
        self.base_frame = p("base_frame", "base_footprint")
        self.map_name = p("map_name", "map")
        self.min_dist = p("min_dist", 0.20)
        self.max_dist = p("max_dist", 1.50)
        self.max_view_angle = math.radians(p("max_view_angle_deg", 45.0))
        self.max_lin = p("max_lin_vel", 0.05)
        self.max_ang = p("max_ang_vel", 0.15)
        self.min_samples = p("min_samples", 15)
        self.visit_gap = p("visit_gap", 3.0)          # 이 시간(s) 이상 안 보이면 새 방문

        self.tf = Buffer(cache_time=Duration(seconds=30))
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.moving = True
        self.start = None
        self.visits = {i: [] for i in MARKERS}        # 방문별 [(시각, (x, y, yaw)), ...]
        self.first_seen = {}
        self.reported = set()

        for i, c in MARKERS.items():
            topic = f"/aruco/{c['name']}_center" if c.get("radius") else f"/aruco/{c['name']}_pose"
            self.create_subscription(PoseStamped, topic, lambda m, i=i: self.marker_cb(i, m), 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.viz = self.create_publisher(MarkerArray, "/marker_map", 1)
        self.create_service(Trigger, "~/save", lambda rq, rs: self.srv(rs, self.save))
        self.create_service(Trigger, "~/status", lambda rq, rs: self.srv(rs, self.status))
        self.create_service(Trigger, "~/set_start", lambda rq, rs: self.srv(rs, self.set_start))
        self.create_timer(1.0, self.publish_viz)
        self.start_timer = self.create_timer(0.5, self.try_init_start)
        self.get_logger().info(
            f"marker_mapper ready ({self.map_frame}). 마커 앞에서 2~3초 멈추면 기록됩니다. "
            f"저장: ros2 service call /marker_mapper/save std_srvs/srv/Trigger")

    # ---------------------------------------------------------------- 입력
    def odom_cb(self, m):
        t = m.twist.twist
        self.moving = abs(t.linear.x) > self.max_lin or abs(t.angular.z) > self.max_ang

    def robot_pose(self):
        tf = self.tf.lookup_transform(self.map_frame, self.base_frame, Time(),
                                      timeout=Duration(seconds=0.1))
        tr, q = tf.transform.translation, tf.transform.rotation
        return (float(tr.x), float(tr.y), float(yaw_of(q)))

    def try_init_start(self):
        try:
            self.start = self.robot_pose()
        except TransformException:
            self.get_logger().info("map TF 기다리는 중 (slam_toolbox 실행 확인)", throttle_duration_sec=5.0)
            return
        self.start_timer.cancel()
        x, y, t = self.start
        self.get_logger().info(f"START = ({x:.2f}, {y:.2f}, {math.degrees(t):.0f}°)")

    def set_start(self):
        self.start = self.robot_pose()
        x, y, t = self.start
        return True, f"START = ({x:.2f}, {y:.2f}, {math.degrees(t):.0f}°)"

    def marker_cb(self, i, msg):
        if self.moving:
            return
        msg.header.stamp = Time().to_msg()           # 멈춰 있을 때만 쓰므로 최신 TF 사용 (시각 동기 오차 무관)
        try:
            pb = self.tf.transform(msg, self.base_frame, timeout=Duration(seconds=0.05))
            pm = self.tf.transform(msg, self.map_frame, timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f"TF 변환 실패: {e}", throttle_duration_sec=3.0)
            return
        bx, by = pb.pose.position.x, pb.pose.position.y
        nbx, nby, _ = quat_z_axis(pb.pose.orientation)
        nmx, nmy, _ = quat_z_axis(pm.pose.orientation)
        if math.hypot(nbx, nby) < 0.5:               # 마커가 위/아래를 향함 → 신뢰 불가
            return
        dist = math.hypot(bx, by)
        if not self.min_dist <= dist <= self.max_dist:
            return
        # 정면 각도: 마커 법선과 '마커 → 로봇' 방향 사이 각
        view = math.acos(max(-1.0, min(1.0, (-bx * nbx - by * nby) / (dist * math.hypot(nbx, nby)))))
        if view > self.max_view_angle:
            return
        now = time.monotonic()
        pose = (float(pm.pose.position.x), float(pm.pose.position.y), float(math.atan2(nmy, nmx)), dist)
        with self.lock:
            v = self.visits[i]
            if not v or now - v[-1][-1][0] > self.visit_gap:
                v.append([])
            v[-1].append((now, pose))
            self.first_seen.setdefault(i, now)
            n = len(v[-1])
        if n == self.min_samples:
            res = self.estimate(i)
            if res:
                (x, y, t), (sd, _), _ = res
                self.get_logger().info(
                    f"[기록] ID {i} {MARKERS[i]['name']}: ({x:.3f}, {y:.3f}) 법선 {math.degrees(t):.0f}° "
                    f"흔들림 {sd * 100:.1f} cm (방문 {len(self.visits[i])}번째)")

    # ---------------------------------------------------------------- 계산 · 저장
    def estimate(self, i):
        """충분히 측정된 방문 중 가장 가까이에서 본 방문(비슷하면 최근 것)의 중앙값."""
        with self.lock:
            visits = [list(v) for v in self.visits[i] if len(v) >= self.min_samples]
        if not visits:
            return None
        dists = [sorted(p[3] for _, p in v)[len(v) // 2] for v in visits]
        best = max(k for k, d in enumerate(dists) if d <= min(dists) + 0.15)
        v = visits[best]
        med, spread = median_pose([p[:3] for _, p in v])
        return med, spread, len(v)

    def status(self):
        lines = []
        for i, c in MARKERS.items():
            r = self.estimate(i)
            if r:
                (x, y, _), (sd, _), n = r
                lines.append(f"ID {i} {c['name']}({c['wp']}): ({x:.2f}, {y:.2f}) {n}개 ±{sd * 100:.1f} cm")
            else:
                n = max((len(v) for v in self.visits[i]), default=0)
                lines.append(f"ID {i} {c['name']}({c['wp']}): 미기록 ({n}/{self.min_samples})")
        s = "START 미정" if self.start is None else "START ({:.2f}, {:.2f})".format(*self.start[:2])
        return True, s + " | " + " | ".join(lines)

    def save(self):
        if self.start is None:
            return False, "START 가 없습니다 (map TF 확인)"
        markers, waypoints = {}, []
        sx, sy, st = self.start
        waypoints.append(dict(name="START", x=round(sx, 3), y=round(sy, 3), yaw=round(math.degrees(st), 1)))
        order = sorted((i for i in MARKERS if self.estimate(i)), key=lambda i: self.first_seen[i])
        if not order:
            return False, "기록된 마커가 없어 저장하지 않았습니다 (기존 파일 유지)"
        for i in order:
            c = MARKERS[i]
            (x, y, yaw), (sd, ysd), n = self.estimate(i)
            markers[c["name"]] = dict(id=i, x=round(x, 4), y=round(y, 4), yaw=round(math.degrees(yaw), 2),
                                      samples=n, spread_cm=round(sd * 100, 2))
            if c.get("radius"):                       # 원통: x, y = 중심축, yaw = 마커가 보이던 방향
                markers[c["name"]]["radius"] = c["radius"]
                dist = c["radius"] + c["dist"]
            else:
                dist = c["dist"]
            wx, wy, wyaw = approach_point(x, y, yaw, dist)
            waypoints.append(dict(name=c["wp"], x=round(wx, 3), y=round(wy, 3),
                                  yaw=round(math.degrees(wyaw), 1), marker=i))

        os.makedirs(os.path.join(SLAM_DIR, "maps"), exist_ok=True)
        mfile = os.path.join(SLAM_DIR, "maps", f"{self.map_name}_markers.yaml")
        wfile = os.path.join(SLAM_DIR, "waypoints.yaml")
        mtext = (f"# marker_mapper {time.strftime('%Y-%m-%d %H:%M')} — 마커 지도 좌표 (frame: map)\n"
                 "# x, y: 평면 마커는 표면 중심, 원통(radius 있음)은 중심축. yaw(도): 마커 법선 방향\n"
                 + yaml.safe_dump(dict(frame=self.map_frame, markers=markers), sort_keys=False, allow_unicode=True))
        wtext = yaml.safe_dump(waypoints, default_flow_style=None, sort_keys=False, allow_unicode=True)
        for path, text in ((mfile, mtext), (wfile, wtext)):
            if os.path.exists(path):
                shutil.copy(path, path + ".bak")
            with open(path, "w", encoding="utf8") as f:
                f.write(text)
        missing = [MARKERS[i]["name"] for i in MARKERS if i not in order]
        msg = f"저장: {mfile}, {wfile} ({' → '.join(w['name'] for w in waypoints)})"
        if missing:
            msg += f" | 미기록: {', '.join(missing)}"
        return True, msg

    def srv(self, res, fn):
        try:
            res.success, res.message = fn()
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, f"{type(e).__name__}: {e}"
        self.get_logger().info(res.message)
        return res

    # ---------------------------------------------------------------- RViz
    def publish_viz(self):
        arr = MarkerArray()
        for i, c in MARKERS.items():
            r = self.estimate(i)
            if not r:
                continue
            (x, y, yaw), _, _ = r
            for k, (typ, scale, z, color) in enumerate([
                    (Marker.ARROW, (0.25, 0.04, 0.04), 0.15, (1.0, 0.5, 0.0)),
                    (Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.12), 0.40, (1.0, 1.0, 1.0))]):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns, m.id, m.type = "aruco", i * 10 + k, typ
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
                m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                m.scale.x, m.scale.y, m.scale.z = scale
                m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
                m.text = f"ID{i} {c['wp']}"
                arr.markers.append(m)
        self.viz.publish(arr)


def main():
    rclpy.init()
    node = MarkerMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ok, msg = node.save()
            print(msg if ok else f"저장 안 함: {msg}")
        except Exception as e:  # noqa: BLE001
            print(f"저장 실패: {e}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
