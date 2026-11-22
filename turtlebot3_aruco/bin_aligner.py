"""쓰레기통 앞 미세 정렬 노드 (Orin에서 실행)

Nav2로 쓰레기통 지점에 도착한 뒤, 몸체에 고정된 Pi 카메라로 본 쓰레기통 마커(/aruco/bin_pose)를
기준으로 로봇을 '마커 정면, standoff 거리'에 맞춘다.

  python3 bin_aligner.py                                       # 서비스 대기
  ros2 service call /bin_aligner/align std_srvs/srv/Trigger    # 정렬 시작 (끝날 때까지 블록)
  python3 bin_aligner.py --ros-args -p auto_start:=true        # 테스트용: 켜자마자 정렬

동작 (와플파이는 옆으로 못 움직이므로 '주차'처럼 3단계로 맞춘다)
  APPROACH : 마커 법선 위의 직선을 따라 목표 거리까지 전진/후진하며 좌우 오차를 줄임
  BACKOFF  : 목표 거리에 왔는데 좌우 오차가 남았으면 backoff 만큼 후진하며 좌우 보정 → 다시 APPROACH
  ROTATE   : 위치가 맞으면 제자리 회전으로 정면 각도 맞춤 → 오차 유지되면 완료

주의
- Nav2 목표가 끝난 뒤에만 호출할 것 (Nav2와 /cmd_vel 충돌)
- base_link -> Pi 카메라 광학 프레임 TF가 있어야 함 (터틀봇3 URDF의 camera_rgb_optical_frame)
- 시뮬레이션 검증 (Nav2 도착 오차 앞뒤·좌우 ±5 cm, 각도 ±10°, 측정 잡음 2~4 mm, 45개 조건):
  45/45 수렴, 평균 약 9 s, 최대 18 s, 오차 중앙값 앞뒤 5 mm · 좌우 1 mm · 각도 1°
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lim):
    return max(-lim, min(lim, v))


def quat_z_axis(q):
    """쿼터니언 회전의 z축 벡터 (= 마커 법선, 마커 밖·카메라 쪽 방향)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


class BinAligner(Node):
    def __init__(self):
        super().__init__("bin_aligner")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.pose_topic = p("pose_topic", "/aruco/bin_pose")
        self.base_frame = p("base_frame", "base_link")
        self.standoff = p("standoff", 0.25)        # base_link 원점 ~ 통 표면 목표 거리 (m)
        self.pos_tol = p("pos_tol", 0.015)         # 앞뒤 허용 오차 (m)
        self.lat_tol = p("lat_tol", 0.015)         # 좌우 허용 오차 (m)
        self.yaw_tol = math.radians(p("yaw_tol_deg", 3.0))
        self.hold_time = p("hold_time", 0.5)       # 허용 오차 유지 시간 (s)
        self.max_v = p("max_v", 0.05)              # m/s
        self.min_v = p("min_v", 0.01)              # 마찰 극복용 최소 직진 속도
        self.max_w = p("max_w", 0.3)               # rad/s
        self.min_w = p("min_w", 0.04)              # 마찰 극복용 최소 회전 속도
        self.k_x = p("k_x", 0.8)                   # 거리 게인
        self.k_lat = p("k_lat", 15.0)              # 좌우 오차 → 목표 기울기 게인 (1/m)
        self.k_psi = p("k_psi", 1.5)               # 기울기 추종 게인
        self.k_yaw = p("k_yaw", 1.2)               # 마지막 제자리 회전 게인
        self.psi_max = math.radians(p("psi_max_deg", 15.0))  # 주행 중 최대 기울기 (마커 시야 유지)
        self.backoff = p("backoff", 0.15)          # 좌우 재보정 시 후진 거리 (m)
        self.max_retries = p("max_retries", 3)
        self.stale = p("stale", 0.3)               # 이 시간 넘게 마커 없으면 일시 정지 (s)
        self.lost_timeout = p("lost_timeout", 2.0) # 이 시간 넘게 마커 없으면 실패 (s)
        self.total_timeout = p("total_timeout", 40.0)
        auto_start = p("auto_start", False)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10, callback_group=cb)
        self.create_timer(0.05, self.control_step, callback_group=cb)  # 20 Hz
        self.create_service(Trigger, "/bin_aligner/align", self.align_srv,
                            callback_group=MutuallyExclusiveCallbackGroup())

        self.lock = threading.Lock()
        self.marker = None          # (mx, my, nx, ny, 수신 시각) — base_link 기준
        self.active = False
        self.done = threading.Event()
        self.result = (False, "")
        if auto_start:
            self.start()
        self.get_logger().info(
            f"ready: standoff={self.standoff} m, tol={self.pos_tol * 1000:.0f}/"
            f"{self.lat_tol * 1000:.0f} mm, {math.degrees(self.yaw_tol):.1f} deg")

    # ---------------------------------------------------------------- 입력
    def pose_cb(self, msg):
        try:
            pb = self.tf.transform(msg, self.base_frame, timeout=Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(f"TF 변환 실패: {e}", throttle_duration_sec=2.0)
            return
        nx, ny, _ = quat_z_axis(pb.pose.orientation)
        norm = math.hypot(nx, ny)
        if norm < 0.3:              # 마커가 거의 위/아래를 향함 → 신뢰 불가
            return
        with self.lock:
            self.marker = (pb.pose.position.x, pb.pose.position.y, nx / norm, ny / norm,
                           time.monotonic())

    # ---------------------------------------------------------------- 서비스
    def start(self):
        with self.lock:
            self.marker = None
        self.t_start = time.monotonic()
        self.t_good = None
        self.phase = "APPROACH"
        self.retries = 0
        self.result = (False, "")
        self.done.clear()
        self.active = True
        self.get_logger().info("정렬 시작")

    def align_srv(self, _req, res):
        if self.active:
            res.success, res.message = False, "이미 정렬 중"
            return res
        self.start()
        self.done.wait(self.total_timeout + 2.0)
        res.success, res.message = self.result
        return res

    def finish(self, ok, msg):
        self.active = False
        self.cmd_pub.publish(Twist())
        self.result = (ok, msg)
        self.done.set()
        (self.get_logger().info if ok else self.get_logger().error)(msg)

    def set_phase(self, phase):
        if phase != self.phase:
            self.get_logger().info(f"{self.phase} -> {phase}")
            self.phase = phase

    # ---------------------------------------------------------------- 제어
    def control_step(self):
        if not self.active:
            return
        now = time.monotonic()
        if now - self.t_start > self.total_timeout:
            return self.finish(False, "시간 초과")

        with self.lock:
            m = self.marker
        if m is None or now - m[4] > self.stale:
            self.cmd_pub.publish(Twist())                     # 안 보이면 일단 정지
            last = m[4] if m else self.t_start
            if now - last > self.lost_timeout:
                return self.finish(False, "마커를 놓침")
            return

        mx, my, nx, ny, _ = m
        # 목표 프레임 G: 원점 = 마커 법선 위 standoff 지점, X축 = 통을 향하는 방향
        gx, gy = mx + nx * self.standoff, my + ny * self.standoff
        gth = math.atan2(-ny, -nx)
        c, s = math.cos(gth), math.sin(gth)
        xr = -(gx * c + gy * s)          # 로봇 앞뒤 오차 (+: 목표보다 통 쪽으로 들어감)
        yr = -(-gx * s + gy * c)         # 로봇 좌우 오차
        psi = -gth                       # 로봇 기울기 (0 = 통을 정면으로 봄)

        # 상태 전이
        if self.phase == "APPROACH" and abs(xr) < 0.4 * self.pos_tol:
            if abs(yr) < 0.6 * self.lat_tol:
                self.set_phase("ROTATE")
            else:
                self.retries += 1
                if self.retries > self.max_retries:
                    return self.finish(False, f"좌우 오차 {yr * 1000:.0f} mm 보정 실패")
                self.set_phase("BACKOFF")
        elif self.phase == "ROTATE" and (abs(xr) > 2 * self.pos_tol or abs(yr) > 2 * self.lat_tol):
            self.set_phase("APPROACH")
        elif self.phase == "BACKOFF" and xr < -self.backoff:
            self.set_phase("APPROACH")

        cmd = Twist()
        if self.phase == "ROTATE":
            if abs(xr) < self.pos_tol and abs(yr) < self.lat_tol and abs(psi) < self.yaw_tol:
                self.cmd_pub.publish(cmd)
                self.t_good = self.t_good or now
                if now - self.t_good >= self.hold_time:
                    self.finish(True, f"정렬 완료 (앞뒤 {xr * 1000:.0f} mm, 좌우 {yr * 1000:.0f} mm, "
                                      f"각도 {math.degrees(psi):.1f} deg)")
                return
            self.t_good = None
            w = -self.k_yaw * psi
            if abs(w) < self.min_w:
                w = math.copysign(self.min_w, w)
            cmd.angular.z = clip(w, self.max_w)
        else:
            self.t_good = None
            if self.phase == "BACKOFF":
                v = -self.max_v
            else:
                v = clip(-self.k_x * xr, self.max_v)
                if abs(v) < self.min_v:
                    v = math.copysign(self.min_v, v)
            # 직선 추종: 좌우 오차를 줄이는 방향으로 기울이되 psi_max 이내 (후진이면 반대 방향)
            sign = 1.0 if v > 0 else -1.0
            psi_des = -sign * clip(math.atan(self.k_lat * yr), self.psi_max)
            cmd.linear.x = v
            cmd.angular.z = clip(self.k_psi * wrap(psi_des - psi), self.max_w)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = BinAligner()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # 종료 시 반드시 정지
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
