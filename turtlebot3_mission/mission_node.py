#!/usr/bin/env python3
"""시연 시나리오 전체를 순서대로 실행하는 상위 노드 (Orin에서 실행)

  출발 → 쓰레기 지점 이동 → 관측 자세 → 캔 인식 → 집기 → 팔 접기
       → 쓰레기통 지점 이동 → 마커 정렬 → 놓기 → 팔 접기 → 출발점 복귀

  python3 mission_node.py                                  # 전체 실행
  python3 mission_node.py go_bin align drop stow           # 원하는 단계만 (리허설용)
  python3 mission_node.py --list                           # 단계 목록

웨이포인트: 11주차 waypoint_nav.py 와 같은 ~/waypoints.yaml 을 쓴다.
  python3 ~/waypoint_nav.py record 로 3곳을 찍은 뒤, 파일에서 이름을 START / TRASH / BIN 으로 바꾸고
  yaw(도)를 도착 방향으로 고친다. BIN 의 yaw 는 '쓰레기통을 정면으로 바라보는 방향'.

필요한 노드 (모두 Orin, 라파에서는 bringup · 카메라)
  Nav2 · AMCL, aruco_node.py, bin_aligner.py, arm_server.py, grasp_node(+ 입력 remap), cola_detector

Ctrl+C: 진행 중인 Nav2 이동을 취소하고 로봇을 정지시킨다.
"""
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

WAYPOINTS = os.path.expanduser("~/waypoints.yaml")
STEPS = ["init", "go_trash", "look", "detect", "pick", "stow",
         "go_bin", "align", "drop", "stow", "go_start"]
STEP_INFO = {
    "init": "출발점 초기 위치 설정 · Nav2 준비 확인",
    "go_trash": "Nav2: 쓰레기 지점(TRASH)으로 이동",
    "look": "팔 관측 자세 (/arm/look)",
    "detect": "YOLO 캔 인식 대기 (안 보이면 제자리 좌우 회전 후 재시도)",
    "pick": "집기 (/grasp/pick), 실패 시 재인식 후 재시도",
    "stow": "팔 접기 (/arm/stow)",
    "go_bin": "Nav2: 쓰레기통 지점(BIN)으로 이동",
    "align": "쓰레기통 마커 정렬 (/bin_aligner/align), 실패 시 BIN 재이동 후 재시도",
    "drop": "놓기 (/drop/place)",
    "go_start": "Nav2: 출발점(START)으로 복귀",
}

NAV_RETRIES = 1             # 이동 실패 시 재시도 횟수
ARRIVAL_TOL = 0.40          # Nav2 성공 후 AMCL 위치로 다시 확인하는 도착 허용 거리 (m)
DETECT_TIMEOUT = 5.0        # 한 방향에서 캔을 기다리는 시간 (s)
DETECT_TURNS = [0.35, -0.70, 0.35]   # 못 찾으면 이만큼(rad) 차례로 돌아보고, 마지막은 원위치
PICK_RETRIES = 1
ALIGN_RETRIES = 1
SERVICE_TIMEOUT = {"/arm/look": 40, "/arm/stow": 40, "/grasp/pick": 60,
                   "/bin_aligner/align": 50, "/drop/place": 90}


def spin_until(node, future, timeout=None):
    """future 완료까지 대기. 짧게 끊어 돌려야 Ctrl+C가 바로 먹는다 (goto_pin.py 와 같은 방식)."""
    start = time.monotonic()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if timeout is not None and time.monotonic() - start > timeout:
            return False
    return True


def spin_for(node, sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def yaw_quat(deg):
    y = math.radians(deg)
    return math.sin(y / 2), math.cos(y / 2)


class Mission(Node):
    def __init__(self, points):
        super().__init__("mission_node")
        self.points = points
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_pose, qos)
        self.create_subscription(PointStamped, "/cola/target_point", self.on_target, 10)
        self.clients = {}
        self.position = None
        self.last_target = 0.0
        self.goal_handle = None

    # ---------------------------------------------------------------- 공통
    def on_pose(self, msg):
        self.position = msg.pose.pose.position

    def on_target(self, _msg):
        self.last_target = time.monotonic()

    def log(self, text):
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def stop_base(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            spin_for(self, 0.05)

    def call(self, name):
        """Trigger 서비스 호출 → (성공 여부, 메시지)."""
        if name not in self.clients:
            self.clients[name] = self.create_client(Trigger, name)
        cli = self.clients[name]
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, f"{name} 서비스가 없음 (노드 실행 여부 확인)"
        fut = cli.call_async(Trigger.Request())
        if not spin_until(self, fut, SERVICE_TIMEOUT.get(name, 60)):
            return False, f"{name} 응답 시간 초과"
        res = fut.result()
        return res.success, res.message

    # ---------------------------------------------------------------- Nav2
    def set_initial_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(p["x"]), float(p["y"])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = yaw_quat(p.get("yaw", 0.0))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2
        cov[35] = math.radians(10) ** 2
        msg.pose.covariance = cov
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            spin_for(self, 0.3)

    def go(self, name):
        p = self.points[name]
        for attempt in range(NAV_RETRIES + 1):
            if attempt:
                self.log("  다시 시도합니다.")
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(p["x"]), float(p["y"])
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_quat(p.get("yaw", 0.0))
            accepted = self.nav.send_goal_async(goal)
            spin_until(self, accepted)
            self.goal_handle = accepted.result()
            if not self.goal_handle.accepted:
                self.log("  Nav2가 목표를 거부했습니다.")
                continue
            result = self.goal_handle.get_result_async()
            spin_until(self, result)
            self.goal_handle = None
            if result.result().status != GoalStatus.STATUS_SUCCEEDED:
                self.log(f"  도착 실패 (status={result.result().status})")
                continue
            spin_for(self, 1.0)          # Nav2 성공 보고를 그대로 믿지 않고 AMCL 위치로 확인
            if self.position is not None:
                d = math.hypot(self.position.x - p["x"], self.position.y - p["y"])
                if d > ARRIVAL_TOL:
                    self.log(f"  Nav2는 도착이라 했지만 목표에서 {d:.2f} m 떨어짐")
                    continue
            return True, f"{name} 도착"
        return False, f"{name}에 도착하지 못함"

    def cancel_nav(self):
        if self.goal_handle is not None:
            spin_until(self, self.goal_handle.cancel_goal_async(), timeout=3.0)
            self.goal_handle = None

    def turn(self, angle, w=0.4):
        """제자리 회전 (오도메트리 없이 시간 기준 — 캔을 다시 찾기 위한 대략적인 회전)."""
        cmd = Twist()
        cmd.angular.z = math.copysign(w, angle)
        end = time.monotonic() + abs(angle) / w
        while time.monotonic() < end:
            self.cmd_pub.publish(cmd)
            spin_for(self, 0.05)
        self.stop_base()
        spin_for(self, 0.5)

    # ---------------------------------------------------------------- 단계
    def step_init(self):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            return False, "Nav2(navigate_to_pose)를 찾지 못함"
        self.set_initial_pose(self.points["START"])
        t0 = time.monotonic()
        while self.position is None and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position is None:
            return False, "AMCL 위치가 잡히지 않음 (RViz 2D Pose Estimate 확인)"
        return True, "초기 위치 설정 완료"

    def step_detect(self):
        for turn in [0.0] + DETECT_TURNS:
            if turn:
                self.log(f"  캔이 안 보여서 {math.degrees(turn):+.0f}° 회전")
                self.turn(turn)
            self.last_target = 0.0
            t0 = time.monotonic()
            hits = 0
            while time.monotonic() - t0 < DETECT_TIMEOUT:
                before = self.last_target
                spin_for(self, 0.2)
                hits = hits + 1 if self.last_target > before else 0
                if hits >= 5:            # 1초 가까이 연속으로 보이면 인식 성공
                    return True, "캔 인식"
        return False, "캔을 찾지 못함"

    def step_pick(self):
        for attempt in range(PICK_RETRIES + 1):
            if attempt:
                self.log("  관측 자세로 돌아가 다시 인식합니다.")
                ok, msg = self.call("/arm/look")
                if not ok:
                    return False, msg
                ok, msg = self.step_detect()
                if not ok:
                    return False, msg
            ok, msg = self.call("/grasp/pick")
            if ok:
                return True, msg
            self.log(f"  집기 실패: {msg}")
        return False, "집기 재시도 모두 실패"

    def step_align(self):
        for attempt in range(ALIGN_RETRIES + 1):
            if attempt:
                self.log("  BIN 지점으로 다시 이동 후 재정렬합니다.")
                ok, msg = self.go("BIN")
                if not ok:
                    return False, msg
            ok, msg = self.call("/bin_aligner/align")
            if ok:
                return True, msg
            self.log(f"  정렬 실패: {msg}")
        return False, "정렬 재시도 모두 실패"

    def run_step(self, step):
        return {
            "init": self.step_init,
            "go_trash": lambda: self.go("TRASH"),
            "look": lambda: self.call("/arm/look"),
            "detect": self.step_detect,
            "pick": self.step_pick,
            "stow": lambda: self.call("/arm/stow"),
            "go_bin": lambda: self.go("BIN"),
            "align": self.step_align,
            "drop": lambda: self.call("/drop/place"),
            "go_start": lambda: self.go("START"),
        }[step]()

    def safe_stop(self):
        """실패 · 중단 시: 이동 취소, 바퀴 정지, 팔 접기 시도."""
        self.cancel_nav()
        self.stop_base()
        ok, msg = self.call("/arm/stow")
        self.log(f"  안전 정지: 팔 접기 {'완료' if ok else '실패 — ' + msg}")


def load_points():
    if not os.path.exists(WAYPOINTS):
        return None, f"{WAYPOINTS} 가 없습니다. waypoint_nav.py record 로 만드세요."
    with open(WAYPOINTS, encoding="utf8") as f:
        pts = {p["name"]: p for p in (yaml.safe_load(f) or [])}
    return pts, ""


def main():
    args = sys.argv[1:]
    if args and args[0] in ("--list", "-l", "help", "-h"):
        print(__doc__)
        for s in dict.fromkeys(STEPS):
            print(f"  {s:9s} {STEP_INFO[s]}")
        return 0
    steps = args or STEPS
    unknown = [s for s in steps if s not in STEP_INFO]
    if unknown:
        print(f"없는 단계: {', '.join(unknown)}  (--list 로 확인)")
        return 2

    points, err = load_points()
    if points is None:
        print(err)
        return 2
    need = {"init": "START", "go_start": "START", "go_trash": "TRASH", "go_bin": "BIN", "align": "BIN"}
    missing = sorted({need[s] for s in steps if s in need} - set(points))
    if missing:
        print(f"{WAYPOINTS} 에 웨이포인트가 없습니다: {', '.join(missing)}")
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Mission(points)
    t_start = time.monotonic()
    code = 0
    try:
        for i, step in enumerate(steps, 1):
            node.log(f"[{i}/{len(steps)}] {step}: {STEP_INFO[step]}")
            t0 = time.monotonic()
            ok, msg = node.run_step(step)
            node.log(f"  {'성공' if ok else '실패'} ({time.monotonic() - t0:.1f} s) {msg}")
            if not ok:
                node.safe_stop()
                code = 1
                break
        else:
            node.log(f"시나리오 완료 ({time.monotonic() - t_start:.0f} s)")
    except KeyboardInterrupt:
        node.log("중단 요청 — 정지합니다.")
        node.safe_stop()
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
