"""SLAM 자율 탐색: ArUco 마커 = 맵의 끝 (Orin에서 slam_toolbox · marker_mapper 와 함께 실행)

  w : 탐색 시작 / 일시정지 후 다시 시작
  s : 일시정지 (정지)
  q : 지금 끝내기 (지도 · 마커 · 웨이포인트 저장)
  키는 터미널 또는 /aruco_explore/key (std_msgs/String) 로 받는다 (화면 없이 쓸 때는 headless_map.py 가 보냄).

동작
  0. 시작하면 팔을 기본 자세(파라미터 arm_pose, 기본 home = 로보티즈 초기 자세)로 천천히 옮긴다.
  1. 제자리 한 바퀴로 주변을 본 뒤, 지도에서 '아는 곳과 모르는 곳의 경계(frontier)'를 찾아 Nav2 로 하나씩 찾아간다.
     (Nav2 가 없으면 auto_map.py 규칙으로 돌아다님)
  2. 아직 안 간 마커가 보이면 멈추고, 마커를 비스듬히 보고 있으면 먼저 마커 정면 0.75 m 지점으로
     오도메트리로 이동해 마커를 바라본 뒤, bin_aligner 의 정렬 로직으로 마커 정면 standoff 까지 다가간다.
  3. 정렬이 끝난 곳 = '맵의 끝'. 3초 멈춰 marker_mapper 가 마커 지도 좌표를 기록하게 하고,
     그 마커 뒤쪽은 더 탐색하지 않는다. 180° 돌아서 탐색 계속. (각 마커 앞 = 웨이포인트)
  4. 갈 수 있는 frontier 가 더 이상 없으면 = 맵핑 완료 (또는 q, 시간 제한)
     → 정지 → 지도 · 마커 · 웨이포인트 저장 → 종료. 마커는 '끝 표시'일 뿐 다 찾을 필요는 없다.

  python3 aruco_explore.py --ros-args -p map_name:=map -p time_limit:=900.0
"""
import math
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 변환 등록)

from bin_aligner import BinAligner
from frontier import choose, frontier_clusters, viewpoints
from marker_geom import quat_z_axis, wrap, yaw_of
from marker_mapper import MARKERS

SLAM_DIR = os.path.expanduser(os.environ.get("SLAM_NAV2", "~/slam_nav2"))
sys.path.insert(0, os.path.join(SLAM_DIR, "scripts"))
from auto_map import STALE_SEC, decide  # noqa: E402  (팀 SLAM 자율 주행 규칙 재사용)


# ---------------------------------------------------------------- 탐색 노드
class Explorer(Node):
    def __init__(self):
        super().__init__("aruco_explore")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.map_name = p("map_name", "map")
        self.targets = [int(i) for i in str(p("targets", "0,1,2,3")).split(",") if i.strip()]
        self.trigger_dist = p("trigger_dist", 1.6)        # 이 거리 안에 마커가 보이면 접근 시작 (m)
        self.trigger_frames = p("trigger_frames", 5)      # 0.7초 안에 이만큼 보여야 함
        self.hold = p("hold", 3.0)                        # 끝 지점에서 멈춰 기록하는 시간 (s)
        self.max_attempts = p("max_attempts", 3)
        self.retry_cooldown = p("retry_cooldown", 15.0)   # 접근 실패한 마커를 다시 시도하기까지 (s)
        self.time_limit = p("time_limit", 900.0)          # 탐색 시간 제한 (s)
        self.turn_speed = p("turn_speed", 0.5)
        self.arm_pose = p("arm_pose", "home")              # 시작 시 팔 자세 (home / ready / none = 안 움직임)
        self.pre_dist = p("pre_dist", 0.75)              # 정렬 전에 갈 '마커 정면' 거리 (m)
        self.pre_view_deg = p("pre_view_deg", 20.0)      # 이보다 비스듬하면 정면 지점으로 먼저 이동
        self.pre_speed = p("pre_speed", 0.08)
        self.pre_lateral = p("pre_lateral", 0.06)        # 법선 직선에서 이만큼 벗어나도 정면 지점으로 먼저 이동
        self.use_nav2 = p("use_nav2", True)               # frontier 를 Nav2 로 찾아감 (없으면 auto_map 주행)
        self.goal_timeout = p("goal_timeout", 90.0)        # frontier 하나에 쓰는 최대 시간 (s)
        self.done_checks = p("done_checks", 4)             # frontier 없음이 이만큼(1초 간격) 이어지면 완료
        self.min_frontier = p("min_frontier_m", 0.50)      # 이보다 작은 경계는 무시 (m)
        self.look_time = p("look_time", 300.0)             # 맵핑 완료 후 못 찾은 마커를 둘러보며 찾는 최대 시간 (s)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.obs = {i: [] for i in self.targets}          # (시각, mx, my, nx, ny) odom 기준
        self.map = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal = None                                  # (handle, result_future, x, y, 보낸 시각)
        self.ends = []                                    # 맵의 끝 마커 (mx, my, nx, ny) map 기준
        self.blacklist = []                               # 실패했거나 두 번 가도 안 채워진 frontier (x, y)
        self.arrived = []                                 # 도착한 frontier 목표 (x, y)
        self.no_front = 0
        self.last_front_t = 0.0

        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "scan_fixed", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.seen = {i: [] for i in self.targets}         # 최근 인식 시각
        for i in self.targets:
            self.create_subscription(PoseStamped, f"/aruco/{MARKERS[i]['name']}_pose",
                                     lambda m, i=i: self.marker_cb(i, m), 10)
        self.save_cli = self.create_client(Trigger, "/marker_mapper/save")
        self.keys = []                                    # /aruco_explore/key 로 받은 키
        self.create_subscription(String, "/aruco_explore/key",
                                 lambda m: self.keys.append(m.data.strip().lower()[:1]), 10)
        self.ranges, self.scan_t, self.yaw, self.pose = None, 0.0, None, None
        self.turn_dir = 0
        self.reached = []                                 # 도달한 마커 (순서)
        self.attempts = {i: 0 for i in self.targets}
        self.failed_at = {i: -1e9 for i in self.targets}
        self.lock = threading.Lock()

    def scan_cb(self, m):
        self.ranges, self.scan_t = list(m.ranges), time.monotonic()

    def odom_cb(self, m):
        self.yaw = yaw_of(m.pose.pose.orientation)
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, self.yaw)

    def marker_cb(self, i, m):
        if m.pose.position.z <= self.trigger_dist:        # 카메라 광학 프레임 z = 앞쪽 거리
            with self.lock:
                self.seen[i].append(time.monotonic())
        m.header.stamp = Time().to_msg()
        try:
            po = self.tf.transform(m, "odom", timeout=Duration(seconds=0.05))
        except TransformException:
            return
        nx, ny, _ = quat_z_axis(po.pose.orientation)
        nn = math.hypot(nx, ny)
        if nn > 0.5:
            with self.lock:
                self.obs[i] = [o for o in self.obs[i] if time.monotonic() - o[0] < 5.0]
                self.obs[i].append((time.monotonic(), po.pose.position.x, po.pose.position.y, nx / nn, ny / nn))

    def marker_odom(self, i):
        """최근 5초 평균 마커 위치·법선 (odom, 마커는 움직이지 않음). 측정이 5개 미만이면 None."""
        with self.lock:
            o = [x for x in self.obs[i] if time.monotonic() - x[0] < 5.0]
        if len(o) < 5:
            return None
        mx, my, nx, ny = (sum(x[k] for x in o) / len(o) for k in range(1, 5))
        nn = math.hypot(nx, ny)
        return mx, my, nx / nn, ny / nn

    def front_clear(self, deg=25):
        if self.ranges is None:
            return 0.0
        n = len(self.ranges)
        vals = [self.ranges[round(d * n / 360.0) % n] for d in range(-deg, deg + 1)]
        vals = [v for v in vals if math.isfinite(v)]
        return min(vals) if vals else 0.0

    def send(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def stop(self):
        for _ in range(3):
            self.send()
            time.sleep(0.03)

    def candidate(self):
        """접근할 마커: 아직 안 갔고, 최근 0.7초에 trigger_frames 이상 보인 것."""
        now = time.monotonic()
        with self.lock:
            for i in self.targets:
                self.seen[i] = [t for t in self.seen[i] if now - t < 0.7]
            for i in self.targets:
                if (i not in self.reached and len(self.seen[i]) >= self.trigger_frames
                        and self.attempts[i] < self.max_attempts
                        and now - self.failed_at[i] > self.retry_cooldown):
                    return i
        return None

    def explore_step(self):
        if self.ranges is None or time.monotonic() - self.scan_t > STALE_SEC:
            self.send()
            return
        v, w, self.turn_dir = decide(self.ranges, self.turn_dir)
        self.send(v, w)

    # ---------------------------------------------------------------- frontier + Nav2
    def robot_map(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def odom_to_map(self, x, y, nx, ny):
        t = self.tf.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=0.2))
        c, s = math.cos(yaw_of(t.transform.rotation)), math.sin(yaw_of(t.transform.rotation))
        tx, ty = t.transform.translation.x, t.transform.translation.y
        return tx + c * x - s * y, ty + s * x + c * y, c * nx - s * ny, s * nx + c * ny

    def frontiers(self):
        m, r = self.map, self.robot_map()
        if m is None or r is None:
            return None, 0, []
        i = m.info
        cl = frontier_clusters(m.data, i.width, i.height, i.resolution, i.origin.position.x,
                               i.origin.position.y, self.min_frontier)
        best, n = choose(cl, r[:2], self.ends, self.blacklist)
        return best, n, cl

    def nav_send(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = self.nav.send_goal_async(g)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 5:
            time.sleep(0.05)
        if not f.done() or not f.result().accepted:
            return False
        h = f.result()
        self.goal = (h, h.get_result_async(), x, y, time.monotonic())
        return True

    def nav_cancel(self):
        if self.goal is None:
            return
        f = self.goal[0].cancel_goal_async()
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 3:
            time.sleep(0.05)
        self.goal = None
        for _ in range(10):                                # Nav2 속도 명령이 완전히 멈출 때까지 정지 명령
            self.send()
            time.sleep(0.1)

    def explore_nav_step(self):
        """frontier 하나씩 Nav2 로. 맵핑 완료면 'done'."""
        now = time.monotonic()
        if self.goal is not None:
            h, res, gx, gy, t0 = self.goal
            if res.done():
                st = res.result().status
                if st != GoalStatus.STATUS_SUCCEEDED:
                    log(f"  frontier ({gx:.2f}, {gy:.2f}) 도달 실패 (status {st}) → 제외")
                    self.blacklist.append((gx, gy))
                else:                                      # 도착했는데 같은 곳 경계가 두 번째로 남으면 제외
                    near = [v for v in self.arrived if math.hypot(v[0] - gx, v[1] - gy) < 0.4]
                    if near:
                        self.blacklist.append((gx, gy))
                    self.arrived.append((gx, gy))
                self.goal = None
            elif now - t0 > self.goal_timeout:
                log(f"  frontier ({gx:.2f}, {gy:.2f}) 시간 초과 → 제외")
                self.nav_cancel()
                self.blacklist.append((gx, gy))
            elif now - self.last_front_t > 2.0:            # 가는 도중 그 경계가 이미 지도에 채워졌으면 다음으로
                self.last_front_t = now
                _, _, cl = self.frontiers()
                if cl is not None and not any(math.hypot(c[0] - gx, c[1] - gy) < 0.5 for c in cl):
                    self.nav_cancel()
            return None
        if now - self.last_front_t < 1.0:
            return None
        self.last_front_t = now
        best, n, cl = self.frontiers()
        if best is None:
            if self.map is None:
                return None
            self.no_front += 1
            log(f"  남은 frontier 없음 ({self.no_front}/{self.done_checks})")
            return "done" if self.no_front >= self.done_checks else None
        self.no_front = 0
        r = self.robot_map()
        x, y = best[0], best[1]
        yaw = math.atan2(y - r[1], x - r[0])
        if self.nav_send(x, y, yaw):
            log(f"  frontier → ({x:.2f}, {y:.2f}) 크기 {best[2]:.1f} m (남은 경계 {n}개)")
        else:
            log(f"  Nav2 가 목표 ({x:.2f}, {y:.2f}) 를 거부 → 제외")
            self.blacklist.append((x, y))
        return None

    def look_step(self, state):
        """맵핑 완료 후 못 찾은 마커 찾기: 지도 곳곳의 관측 지점에 가서 90°씩 돌며 본다.
        state: {'pts': [...], 'phase': 'go'|'spin', 'k': 회전 횟수, 't': 시각}. 끝나면 'done'."""
        if state.get("pts") is None:
            m = self.map
            i = m.info
            state["pts"] = viewpoints(m.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
            state["phase"] = None
            log(f"  관측 지점 {len(state['pts'])}곳")
        if self.goal is not None:
            h, res, gx, gy, t0 = self.goal
            if res.done() or time.monotonic() - t0 > self.goal_timeout:
                if not res.done():
                    self.nav_cancel()
                self.goal = None
                state["phase"], state["k"] = "spin", 0
            return None
        if state.get("phase") == "spin":
            return "spin"                                  # 회전은 run() 에서 (키 · 마커 확인하며)
        r = self.robot_map()
        if not state["pts"] or r is None:
            return "done"
        state["pts"].sort(key=lambda q: math.hypot(q[0] - r[0], q[1] - r[1]))
        x, y = state["pts"].pop(0)
        if not self.nav_send(x, y, math.atan2(y - r[1], x - r[0])):
            return None
        log(f"  관측 지점 ({x:.2f}, {y:.2f}) 으로 (남은 {len(state['pts'])}곳)")
        return None

    def wander_step(self):
        """Nav2 없을 때: auto_map 규칙으로 주행 + 1초마다 완료 확인."""
        self.explore_step()
        now = time.monotonic()
        if now - self.last_front_t < 1.0:
            return None
        self.last_front_t = now
        best, _, _ = self.frontiers()
        if best is None and self.map is not None:
            self.no_front += 1
            return "done" if self.no_front >= self.done_checks * 3 else None
        self.no_front = 0
        return None


class KeyReader:
    """터미널(엔터 없이) 또는 /aruco_explore/key 토픽에서 키 하나씩 읽기."""

    def __init__(self, node):
        self.node = node
        self.tty = sys.stdin.isatty()
        if self.tty:
            self.old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def get(self):
        if self.node.keys:
            return self.node.keys.pop(0)
        if self.tty and select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def restore(self):
        if self.tty:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)


def log(text):
    print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)


def run(node, aligners, keys):
    def sleep(sec, check_keys=True):
        """sec 동안 대기. q 를 누르면 True."""
        end = time.monotonic() + sec
        while time.monotonic() < end:
            k = keys.get() if check_keys else None
            if k == "q":
                return True
            if k == "s":
                return "pause"
            time.sleep(0.05)
        return False

    def turn(angle):
        """오도메트리로 제자리 회전. q/s 로 중단 가능."""
        while node.yaw is None:
            time.sleep(0.05)
        start, last, turned = node.yaw, node.yaw, 0.0
        t0 = time.monotonic()
        while abs(turned) < abs(angle) - math.radians(5) and time.monotonic() - t0 < 20:
            if keys.get() in ("q", "s"):
                break
            node.send(0.0, math.copysign(node.turn_speed, angle))
            time.sleep(0.05)
            turned += wrap(node.yaw - last)
            last = node.yaw
        node.stop()
        return wrap(node.yaw - start)

    if node.arm_pose and node.arm_pose != "none":
        log(f"팔을 기본 자세({node.arm_pose})로 옮깁니다 (약 6초)")
        r = subprocess.run(["ros2", "run", "omx_cola_grasp", "arm_pose", node.arm_pose, "--time", "6"],
                           capture_output=True, text=True, timeout=60)
        log("  " + (r.stdout.strip().splitlines() or ["(출력 없음)"])[-1] if r.returncode == 0
            else f"  팔 이동 실패 (계속 진행): {r.stderr.strip()[-300:]}")
    log("준비 완료. [w] 탐색 시작   [s] 일시정지   [q] 끝내고 저장")
    if not keys.tty:
        log("터미널 입력이 아니라서 바로 시작합니다 (키는 /aruco_explore/key).")
    running = not keys.tty
    nav_ok = node.use_nav2 and node.nav.wait_for_server(timeout_sec=20.0)
    log("Nav2 로 frontier 탐색" if nav_ok else "Nav2 없음 → auto_map 규칙으로 탐색")
    spun = False
    look = None                       # 맵핑 완료 후 마커 찾기 상태
    t_run = 0.0                       # 실제로 탐색한 누적 시간
    last = time.monotonic()
    while rclpy.ok():
        k = keys.get()
        if k == "q":
            break
        if k == "w" and not running:
            running = True
            log(f"탐색 시작 — 목표 마커 {node.targets}, 도달 {node.reached}")
        if k == "s" and running:
            running = False
            node.nav_cancel()
            node.stop()
            log("일시정지 ([w] 로 다시 시작)")
        now = time.monotonic()
        if running:
            t_run += now - last
        last = now
        if not running:
            time.sleep(0.05)
            continue
        if t_run > node.time_limit:
            log(f"시간 제한 {node.time_limit:.0f} s 도달")
            break
        if spun and nav_ok is False and node.use_nav2 and node.nav.server_is_ready():
            nav_ok = True
            log("Nav2 연결됨 → frontier 탐색으로 전환")

        if not spun:                  # 처음: 90°씩 돌며 잠깐 멈춰 주변 지도 · 마커 확인 (뒤에 있는 마커도 찾기)
            log("제자리 한 바퀴 (90°씩 멈추며) 주변 확인")
            for _ in range(4):
                if sleep(1.5) or node.candidate() is not None:
                    break
                turn(math.pi / 2)
            spun = True
            continue

        i = node.candidate()
        if i is None:
            if look is not None:                                  # 맵핑 완료 → 못 찾은 마커 둘러보기
                left = [t for t in node.targets if t not in node.reached]
                if not left or time.monotonic() - look["t0"] > node.look_time:
                    log("마커 찾기 끝" + (f" — 못 찾은 마커 ID {left}" if left else " — 모든 마커 찾음"))
                    break
                r = node.look_step(look)
                if r == "done":
                    log(f"관측 지점을 다 돌았습니다 — 못 찾은 마커 ID {left}")
                    break
                if r == "spin":
                    if sleep(1.2) is True:
                        break
                    if node.candidate() is None:
                        look["k"] += 1
                        if look["k"] >= 4:
                            look["phase"] = None
                        else:
                            turn(math.pi / 2)
                time.sleep(0.1)
                continue
            if (node.explore_nav_step() if nav_ok else node.wander_step()) == "done":
                log("★ 갈 수 있는 곳을 다 봤습니다 → 맵핑 완료")
                left = [t for t in node.targets if t not in node.reached]
                if not (nav_ok and left and node.look_time > 0):
                    break
                log(f"아직 못 찾은 마커 ID {left} → 최대 {node.look_time:.0f}초 동안 둘러보며 찾기")
                look = {"t0": time.monotonic()}
            time.sleep(0.1)
            continue

        # ---- 마커 접근 (bin_aligner 정렬 로직)
        name = MARKERS[i]["name"]
        node.nav_cancel()
        node.stop()
        node.attempts[i] += 1
        log(f"ID {i} ({name}) 발견 → 마커 앞으로 접근 ({node.attempts[i]}/{node.max_attempts}회째)")
        pre = pre_approach(node, i, keys)
        if pre in ("q", "s"):
            node.stop()
            if pre == "q":
                break
            running = False
            log("일시정지 ([w] 로 다시 시작)")
            continue
        if pre is not True:
            node.failed_at[i] = time.monotonic()
            log(f"  정면 이동 실패: {pre} → 돌아서 탐색 계속")
            turn(math.radians(90))
            continue
        al = aligners[i]
        al.start()
        stop_req = None
        while not al.done.is_set():
            k = keys.get()
            if k in ("q", "s"):
                stop_req = k
                al.finish(False, "사용자 중단")
                break
            time.sleep(0.05)
        node.stop()
        ok, msg = al.result
        if stop_req == "q":
            break
        if stop_req == "s":
            running = False
            log("일시정지 ([w] 로 다시 시작)")
            continue
        if not ok:
            node.failed_at[i] = time.monotonic()
            log(f"  접근 실패: {msg} → 돌아서 탐색 계속")
            turn(math.radians(90))
            continue

        # ---- 맵의 끝 도달
        log(f"  ★ 맵의 끝 도달: ID {i} ({name}) — {msg}. {node.hold:.0f}초 멈춰 기록합니다.")
        r = sleep(node.hold)
        node.reached.append(i)
        m = node.marker_odom(i)
        if m:
            try:
                node.ends.append(node.odom_to_map(*m))     # 이 마커 뒤쪽은 탐색 안 함
            except TransformException:
                pass
        log(f"  맵의 끝 {len(node.reached)}곳: {node.reached} (이 마커 뒤쪽은 탐색하지 않음)")
        if r is True:
            break
        if r == "pause":
            running = False
            log("일시정지 ([w] 로 다시 시작)")
            continue
        log("  180° 돌아서 탐색 계속")
        turn(math.pi)
    node.nav_cancel()
    node.stop()


def pre_approach(node, i, keys):
    """마커를 비스듬히 보면 마커 정면 pre_dist 지점으로 이동(오도메트리) 후 마커를 바라본다.
    True / 실패 사유 문자열 / 'q'·'s'(사용자 중단)."""
    m, t0 = None, time.monotonic()
    while m is None and time.monotonic() - t0 < 2.5:  # 멈춘 상태에서 측정 모으기
        time.sleep(0.5)
        m = node.marker_odom(i)
    if m is None or node.pose is None:
        return "마커 위치를 못 구함"
    mx, my, nx, ny = m

    def rotate_to(heading):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            k = keys.get()
            if k in ("q", "s"):
                return k
            err = wrap(heading - node.pose[2])
            if abs(err) < math.radians(3):
                break
            node.send(0.0, max(-node.turn_speed, min(node.turn_speed, 1.5 * err)))
            time.sleep(0.05)
        node.stop()
        return True

    x, y, _ = node.pose
    dx, dy = x - mx, y - my
    dist = math.hypot(dx, dy)
    view = math.degrees(math.acos(max(-1.0, min(1.0, (dx * nx + dy * ny) / max(dist, 1e-6)))))
    lateral = abs(dx * ny - dy * nx)                  # 마커 법선 직선에서 좌우로 벗어난 거리
    if view > node.pre_view_deg or lateral > node.pre_lateral:
        px, py = mx + nx * node.pre_dist, my + ny * node.pre_dist
        log(f"  비스듬함({view:.0f}°, 좌우 {lateral:.2f} m) → 마커 정면 {node.pre_dist:.2f} m 지점으로 먼저 이동")
        r = rotate_to(math.atan2(py - y, px - x))
        if r is not True:
            return r
        t0 = time.monotonic()
        limit = 3 * math.hypot(px - x, py - y) / node.pre_speed + 10
        while True:
            k = keys.get()
            if k in ("q", "s"):
                node.stop()
                return k
            x, y, yaw = node.pose
            d = math.hypot(px - x, py - y)
            if d < 0.04:
                break
            if time.monotonic() - t0 > limit:
                node.stop()
                return "정면 지점 이동 시간 초과"
            if node.front_clear() < 0.25:
                node.stop()
                return "앞에 장애물"
            herr = wrap(math.atan2(py - y, px - x) - yaw)
            if abs(herr) > math.radians(60):          # 지나쳤거나 크게 틀어짐 → 그 자리에서 마무리
                break
            node.send(min(node.pre_speed, 0.6 * d + 0.02), 1.5 * herr)
            time.sleep(0.05)
        node.stop()
    x, y, _ = node.pose
    return rotate_to(math.atan2(my - y, mx - x))      # 마커 바라보기 → 이후 bin_aligner


def finish(node):
    log("탐색 종료 → 저장합니다.")
    if node.save_cli.wait_for_service(timeout_sec=3.0):
        fut = node.save_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10:
            time.sleep(0.05)
        if fut.done():
            log(f"  마커·웨이포인트: {fut.result().message}")
    else:
        log("  /marker_mapper/save 없음 (marker_mapper 실행 확인)")
    save_map = os.path.join(SLAM_DIR, "save_map.sh")
    for attempt in range(1, 4):                       # 지도 저장은 가끔 시간 초과 → 최대 3번
        r = subprocess.run([save_map, node.map_name], capture_output=True, text=True, timeout=60)
        ok = r.returncode == 0
        log(f"  지도: {os.path.join(SLAM_DIR, 'maps', node.map_name)}.yaml "
            + ("저장 완료" if ok else f"저장 실패 ({attempt}/3)\n{r.stdout[-300:]}{r.stderr[-300:]}"))
        if ok:
            break
        time.sleep(2)
    wfile = os.path.join(SLAM_DIR, "waypoints.yaml")
    if os.path.exists(wfile):
        print(open(wfile, encoding="utf8").read(), flush=True)
    log("맵핑 끝!")


def main():
    rclpy.init()
    node = Explorer()
    aligners = {}
    for i in node.targets:
        name = MARKERS[i]["name"]
        aligners[i] = BinAligner(f"explore_align_{name}", {
            "pose_topic": f"/aruco/{name}_pose", "service": f"/aruco_explore/align_{name}",
            "standoff": 0.35, "advance": 0.0, "max_v": 0.08, "total_timeout": 60.0,
            "lost_timeout": 3.0, "pos_tol": 0.04, "lat_tol": 0.05, "yaw_tol_deg": 5.0,
            "max_retries": 5})
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    for a in aligners.values():
        ex.add_node(a)
    threading.Thread(target=ex.spin, daemon=True).start()
    keys = KeyReader(node)
    try:
        run(node, aligners, keys)
        if rclpy.ok():                 # Ctrl+C 로 끊은 경우는 저장 없이 종료 (marker_mapper 는 자체 저장)
            node.stop()
            finish(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        keys.restore()
        for a in aligners.values():
            a.active = False
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
