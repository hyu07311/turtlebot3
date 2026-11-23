"""ArUco 마커로 AMCL 위치 보정 (Orin에서 Nav2 와 함께 실행)

맵핑 때 marker_mapper 가 저장한 마커 지도 좌표(~/slam_nav2/maps/map_markers.yaml)와
라파가 보내는 /aruco/* (로봇에서 본 마커 위치)를 비교해 지도상의 로봇 위치를 계산하고,
AMCL 위치와 차이가 나면 /initialpose 로 AMCL 을 바로잡는다.

  python3 aruco_localizer.py                                   # 자동 보정 (멈춰 있을 때만)
  python3 aruco_localizer.py --ros-args -p markers_file:=~/slam_nav2/maps/이름_markers.yaml
  ros2 service call /aruco_localizer/correct std_srvs/srv/Trigger   # 지금 바로 보정 (mission_node 가 마커 지점 도착 시 호출)

보정 방법
  - 평면 마커(ID 1, 2, 3): 마커 위치 + 마커 방향으로 로봇 x, y, yaw 계산.
    yaw 는 마커 한 장의 방향 추정이 라이다보다 덜 정확할 수 있어 AMCL yaw 와 섞는다 (yaw_weight, 기본 0.5).
  - 원통 쓰레기통(ID 0): 중심축 위치만 확실하므로 AMCL yaw 를 그대로 쓰고 x, y 만 보정.
  - 로봇이 멈춰 있을 때, 1초 동안 8프레임 이상 일정하게(흔들림 2 cm, 3° 이하) 보일 때만 보정한다.
  - 차이가 3 cm / 2° 미만이면 그대로 두고, 1 m / 30° 넘게 차이 나면 잘못 본 것으로 보고 무시한다.
"""
import math
import os
import threading
import time
from collections import deque

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)

from marker_geom import median_pose, quat_z_axis, robot_from_point, wrap, yaw_of

SLAM_DIR = os.path.expanduser(os.environ.get("SLAM_NAV2", "~/slam_nav2"))


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__("aruco_localizer")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.markers_file = os.path.expanduser(p("markers_file", os.path.join(SLAM_DIR, "maps", "map_markers.yaml")))
        self.map_frame = p("map_frame", "map")
        self.base_frame = p("base_frame", "base_footprint")
        self.auto = p("auto", True)
        self.yaw_weight = p("yaw_weight", 0.5)
        self.min_dist = p("min_dist", 0.20)
        self.max_dist = p("max_dist", 1.50)
        self.max_view_angle = math.radians(p("max_view_angle_deg", 50.0))
        self.max_lin = p("max_lin_vel", 0.03)
        self.max_ang = p("max_ang_vel", 0.10)
        self.window = p("window_sec", 1.0)
        self.min_samples = p("min_samples", 8)
        self.max_spread = p("max_spread", 0.02)
        self.max_yaw_spread = math.radians(p("max_yaw_spread_deg", 3.0))
        self.min_corr = p("min_correction", 0.03)
        self.min_yaw_corr = math.radians(p("min_yaw_correction_deg", 2.0))
        self.max_jump = p("max_jump", 1.0)
        self.max_yaw_jump = math.radians(p("max_yaw_jump_deg", 30.0))
        self.cooldown = p("cooldown", 4.0)
        self.cov_xy = p("cov_xy", 0.05)
        self.cov_yaw = math.radians(p("cov_yaw_deg", 3.0))

        with open(self.markers_file, encoding="utf8") as f:
            data = yaml.safe_load(f) or {}
        self.map = {}
        for name, m in (data.get("markers") or {}).items():
            self.map[name] = dict(m, yaw=math.radians(m["yaw"]))

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.samples = deque(maxlen=200)       # (시각, 이름, bx, by, byaw)
        self.amcl = None                        # (x, y, yaw, 시각)
        self.moving = True
        self.last_corr = 0.0
        self.force = None                       # 서비스 요청: {"event", "result"}

        cb = ReentrantCallbackGroup()
        for name, m in self.map.items():
            topic = f"/aruco/{name}_center" if m.get("radius") else f"/aruco/{name}_pose"
            self.create_subscription(PoseStamped, topic, lambda msg, n=name: self.marker_cb(n, msg), 10,
                                     callback_group=cb)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_cb, 10, callback_group=cb)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.est_pub = self.create_publisher(PoseStamped, "~/robot_pose", 10)
        self.create_timer(0.2, self.step, callback_group=cb)
        self.create_service(Trigger, "~/correct", self.correct_srv,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f"aruco_localizer ready: {self.markers_file} → "
            + ", ".join(f"{n}(ID {m['id']})" for n, m in self.map.items()) + f", auto={self.auto}")

    # ---------------------------------------------------------------- 입력
    def odom_cb(self, m):
        t = m.twist.twist
        self.moving = abs(t.linear.x) > self.max_lin or abs(t.angular.z) > self.max_ang

    def amcl_cb(self, m):
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())

    def marker_cb(self, name, msg):
        if self.moving:
            return
        msg.header.stamp = Time().to_msg()      # base ← 카메라는 고정 TF 이므로 최신 값 사용
        try:
            pb = self.tf.transform(msg, self.base_frame, timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f"TF 변환 실패: {e}", throttle_duration_sec=3.0)
            return
        bx, by = float(pb.pose.position.x), float(pb.pose.position.y)
        nx, ny, _ = quat_z_axis(pb.pose.orientation)
        nn = math.hypot(nx, ny)
        dist = math.hypot(bx, by)
        if nn < 0.5 or not self.min_dist <= dist <= self.max_dist:
            return
        view = math.acos(max(-1.0, min(1.0, (-bx * nx - by * ny) / (dist * nn))))
        if view > self.max_view_angle:
            return
        with self.lock:
            self.samples.append((time.monotonic(), name, bx, by, math.atan2(ny, nx)))

    # ---------------------------------------------------------------- 계산
    def estimate(self):
        """최근 window 초 동안 가장 많이 보인 마커로 로봇 위치 추정. (pose, 이름, 개수, 흔들림) 또는 사유 문자열."""
        if self.amcl is None:
            return "AMCL 위치(/amcl_pose) 없음"
        now = time.monotonic()
        with self.lock:
            recent = [s for s in self.samples if now - s[0] <= self.window]
        if not recent:
            return "보이는 마커 없음"
        names = [s[1] for s in recent]
        # 충분히 보인 마커 중 방향까지 알 수 있는 평면 마커를 원통(쓰레기통)보다 우선, 그다음 많이 보인 순
        name = max(set(names), key=lambda n: (names.count(n) >= self.min_samples,
                                              not self.map[n].get("radius"), names.count(n)))
        obs = [s for s in recent if s[1] == name]
        if len(obs) < self.min_samples:
            return f"{name} 측정 부족 ({len(obs)}/{self.min_samples})"
        m = self.map[name]
        ax, ay, ayaw, _ = self.amcl
        poses = []
        for _, _, bx, by, byaw in obs:
            if m.get("radius"):
                yaw = ayaw                                       # 원통: 방향 정보 없음 → AMCL yaw
            else:
                yaw_m = wrap(m["yaw"] - byaw)                    # 마커 방향으로 본 로봇 yaw
                yaw = wrap(ayaw + self.yaw_weight * wrap(yaw_m - ayaw))
            poses.append(robot_from_point((m["x"], m["y"]), (bx, by), yaw))
        # yaw 흔들림은 마커 방향 측정값 자체로 판단
        yaw_meas = [(0.0, 0.0, wrap(m["yaw"] - o[4])) for o in obs]
        pose, (sd, _) = median_pose(poses)
        _, (_, ysd) = median_pose(yaw_meas)
        if sd > self.max_spread or (not m.get("radius") and ysd > self.max_yaw_spread):
            return f"{name} 측정이 흔들림 ({sd * 100:.1f} cm, {math.degrees(ysd):.1f}°)"
        return pose, name, len(obs), sd

    def step(self):
        force = self.force
        if not (self.auto or force) or self.moving:
            return
        if not force and time.monotonic() - self.last_corr < self.cooldown:
            return
        r = self.estimate()
        if isinstance(r, str):
            return
        (x, y, yaw), name, n, sd = r
        ax, ay, ayaw, _ = self.amcl
        dpos, dyaw = math.hypot(x - ax, y - ay), abs(wrap(yaw - ayaw))
        self.publish_est(x, y, yaw)
        info = (f"ID {self.map[name]['id']} {name}: AMCL ({ax:.2f}, {ay:.2f}, {math.degrees(ayaw):.0f}°) → "
                f"({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°), 차이 {dpos * 100:.1f} cm / {math.degrees(dyaw):.1f}°")
        if dpos > self.max_jump or dyaw > self.max_yaw_jump:
            self.get_logger().warn(f"차이가 너무 커서 무시 — {info}", throttle_duration_sec=3.0)
            self.finish(force, False, "차이가 너무 큼(잘못 인식?) — " + info)
            return
        if not force and dpos < self.min_corr and dyaw < self.min_yaw_corr:
            return                                               # 이미 정확함
        self.publish_initialpose(x, y, yaw)
        self.last_corr = time.monotonic()
        self.get_logger().info(f"[보정] {info} ({n}프레임, ±{sd * 100:.1f} cm)")
        self.finish(force, True, "보정 " + info)

    def finish(self, force, ok, msg):
        if force and not force["event"].is_set():
            force["result"] = (ok, msg)
            force["event"].set()

    def publish_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.cov_xy ** 2
        cov[35] = self.cov_yaw ** 2
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    def publish_est(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.est_pub.publish(m)

    # ---------------------------------------------------------------- 서비스
    def correct_srv(self, _req, res):
        """멈춘 상태에서 마커가 보이면 차이가 작아도 바로 보정. 최대 timeout 초 대기."""
        timeout = 4.0
        req = {"event": threading.Event(), "result": None}
        self.force = req
        if not req["event"].wait(timeout):
            r = self.estimate()
            res.success, res.message = False, f"{timeout:.0f}초 안에 보정 못 함: " + (r if isinstance(r, str) else "로봇이 움직이는 중")
        else:
            res.success, res.message = req["result"]
        self.force = None
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = ArucoLocalizer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
