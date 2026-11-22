"""팔 동작 서비스 서버 (Orin에서 실행) — mission_node가 호출하는 팔 쪽 창구

  /arm/look    (Trigger)  관측 자세: 팔에 달린 D435i가 앞쪽 바닥을 내려다보게
  /arm/stow    (Trigger)  접은 자세: 주행 · 정렬 중 (Pi 카메라 시야 가림 · 충돌 방지)
  /grasp/pick  (Trigger)  기존 grasp_node 감싸기: 안정된 YOLO 목표 1개를 넘기고 완료를 기다림
  /drop/place  (Trigger)  쓰레기통 위로 팔 이동 → 그리퍼 열기 → 복귀

  python3 arm_server.py
  python3 arm_server.py --ros-args -p look_pose:="[0.0, -0.3, 0.5, 1.1]"

사용하는 인터페이스 (⚠ 설치된 turtlebot3_manipulation 설정과 이름이 같은지 확인)
  MoveIt   : /move_action (moveit_msgs/action/MoveGroup), 그룹 "arm", 관절 joint1~4
  그리퍼   : /gripper_controller/gripper_cmd (control_msgs/action/GripperCommand)
  관절 상태 : /joint_states (팀 가이드 9장처럼 팔 쪽 이름을 분리했다면 joint_states_topic 수정)

grasp_node 연동 (grasp_node 코드는 그대로, 실행 인자와 2줄만 추가)
  1) grasp_node 실행 시 입력 토픽을 바꿔서 YOLO가 직접 집기를 시작하지 않게 한다
       ros2 run omx_cola_grasp grasp_node --ros-args -r /cola/target_point:=/grasp/target -p execute:=true
  2) grasp_node의 집기 순서(열기→접근→집기→닫기→들기)가 끝나는 곳에서 결과를 발행
       self.result_pub = self.create_publisher(Bool, '/grasp/result', 10)   # __init__
       self.result_pub.publish(Bool(data=success))                          # 집기 끝
  2)를 아직 못 했으면 grasp_result_topic:="" 로 두면 '팔이 움직였다 멈춤'으로 완료를 추정한다.
"""
import math
import statistics
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401

from omx_ik import ik, max_reach

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]


def wait_future(fut, timeout):
    """다른 스레드(executor)가 future를 완료시킬 때까지 대기."""
    ev = threading.Event()
    fut.add_done_callback(lambda _f: ev.set())
    if not ev.wait(timeout):
        return None
    return fut.result()


class ArmServer(Node):
    def __init__(self):
        super().__init__("arm_server")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        # ⚠ 자세는 RViz MotionPlanning 에서 직접 맞춰 보고 관절값을 옮겨 적을 것
        self.look_pose = p("look_pose", [0.0, -0.3, 0.5, 1.1])
        self.stow_pose = p("stow_pose", [0.0, -1.0, 0.3, 0.7])
        self.group = p("group", "arm")
        self.vel_scale = p("velocity_scale", 0.3)
        self.gripper_open = p("gripper_open", 0.019)      # ⚠ 그리퍼 관절 열림 위치 (m)
        self.gripper_joint = p("gripper_joint", "gripper_left_joint")
        self.empty_close = p("gripper_empty_below", 0.0)  # 닫은 뒤 이보다 작으면 '빈손' 판정 (⚠ 실측)
        self.joint_states_topic = p("joint_states_topic", "/joint_states")
        # grasp_node 연동
        self.target_topic = p("target_topic", "/cola/target_point")
        self.grasp_target_topic = p("grasp_target_topic", "/grasp/target")
        self.grasp_result_topic = p("grasp_result_topic", "/grasp/result")
        self.grasp_frame = p("grasp_frame", "link1")
        self.pick_timeout = p("pick_timeout", 40.0)
        # 놓기
        self.bin_topic = p("bin_topic", "/aruco/bin_center")
        self.floor_frame = p("floor_frame", "base_footprint")
        self.bin_height = p("bin_height", 0.20)           # 통 높이 (m)
        self.bin_radius = p("bin_radius", 0.095)
        self.drop_height = p("drop_height", 0.30)         # 놓을 때 그리퍼 중심 높이 (바닥 기준, m)
        self.drop_lift = p("drop_lift", 0.05)             # 통 위로 갈 때 추가로 띄우는 높이 (m)
        self.drop_pitches = p("drop_pitch_candidates_deg", [0.0, 10.0, 20.0, 30.0, -10.0])
        self.odom_frame = p("odom_frame", "odom")
        self.bin_memory = p("bin_memory", 60.0)   # 마지막으로 본 통 위치를 쓸 수 있는 시간 (s)

        cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.move = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)
        self.grip = ActionClient(self, GripperCommand,
                                 p("gripper_action", "/gripper_controller/gripper_cmd"), callback_group=cb)
        self.grasp_target_pub = self.create_publisher(PointStamped, self.grasp_target_topic, 10)
        self.co_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        self.lock = threading.Lock()
        self.joints = {}
        self.joint_t = 0.0
        self.targets = []           # 최근 YOLO 목표 [(t, x, y, z)] — grasp_frame 기준
        self.bin = None             # (t, PoseStamped) — odom 기준 (로봇이 움직여도 통 위치 유지)
        self.grasp_result = None
        self.create_subscription(JointState, self.joint_states_topic, self.on_joints, 10, callback_group=cb)
        self.create_subscription(PointStamped, self.target_topic, self.on_target, 10, callback_group=cb)
        self.create_subscription(PoseStamped, self.bin_topic, self.on_bin, 10, callback_group=cb)
        if self.grasp_result_topic:
            self.create_subscription(Bool, self.grasp_result_topic, self.on_grasp_result, 10,
                                     callback_group=cb)

        for name, fn in [("/arm/look", self.srv_look), ("/arm/stow", self.srv_stow),
                         ("/grasp/pick", self.srv_pick), ("/drop/place", self.srv_drop)]:
            self.create_service(Trigger, name, fn, callback_group=cb)
        self.busy = threading.Lock()   # 팔 동작은 한 번에 하나만
        self.get_logger().info("arm_server ready")

    # ------------------------------------------------------------ 구독
    def on_joints(self, msg):
        with self.lock:
            for n, v in zip(msg.name, msg.position):
                self.joints[n] = v
            self.joint_t = time.monotonic()

    def on_target(self, msg):
        try:
            pt = self.tf.transform(msg, self.grasp_frame, timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f"목표 TF 변환 실패: {e}", throttle_duration_sec=2.0)
            return
        with self.lock:
            now = time.monotonic()
            self.targets = [t for t in self.targets if now - t[0] < 2.0]
            self.targets.append((now, pt.point.x, pt.point.y, pt.point.z))

    def on_bin(self, msg):
        # odom 기준으로 바꿔 저장 → 정렬 후 직진(advance)으로 마커가 안 보여도 통 위치를 쓸 수 있다
        try:
            m = self.tf.transform(msg, self.odom_frame, timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f"bin TF 변환 실패: {e}", throttle_duration_sec=2.0)
            return
        m.header.stamp = Time().to_msg()   # 이후 변환은 최신 TF 기준
        with self.lock:
            self.bin = (time.monotonic(), m)

    def on_grasp_result(self, msg):
        self.grasp_result = msg.data

    # ------------------------------------------------------------ 저수준 동작
    def move_joints(self, q, timeout=30.0):
        if not self.move.wait_for_server(timeout_sec=5.0):
            return False, "MoveIt(/move_action)를 찾지 못함"
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0) for n, v in zip(ARM_JOINTS, q)])]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = wait_future(self.move.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False, "MoveIt이 목표를 거부"
        res = wait_future(handle.get_result_async(), timeout)
        if res is None:
            handle.cancel_goal_async()
            return False, "MoveIt 실행 시간 초과"
        code = res.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return False, f"MoveIt 실패 (error_code {code})"
        return True, "ok"

    def gripper(self, position, effort=1.0):
        if not self.grip.wait_for_server(timeout_sec=5.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        handle = wait_future(self.grip.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False
        res = wait_future(handle.get_result_async(), 10.0)
        return res is not None   # 물체에 막혀 stalled 로 끝나도 동작 자체는 정상

    def bin_object(self, op, x=0.0, y=0.0):
        """MoveIt 계획 장면에 쓰레기통(원통)을 추가/제거 → 팔이 통 테두리에 부딪히지 않게 계획."""
        co = CollisionObject()
        co.header.frame_id = self.floor_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = "trash_bin"
        co.operation = op
        if op == CollisionObject.ADD:
            co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                            dimensions=[self.bin_height, self.bin_radius])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, self.bin_height / 2
            pose.orientation.w = 1.0
            co.primitive_poses = [pose]
        self.co_pub.publish(co)
        time.sleep(0.3)

    def arm_settled_after_motion(self, timeout):
        """grasp_node 결과 토픽이 없을 때: 팔이 움직이기 시작했다가 1.5 s 멈추면 완료로 본다."""
        def snap():
            with self.lock:
                return [self.joints.get(n, 0.0) for n in ARM_JOINTS + [self.gripper_joint]]
        t0, start, moved = time.monotonic(), snap(), False
        last, still_since = start, None
        while time.monotonic() - t0 < timeout:
            time.sleep(0.1)
            cur = snap()
            if not moved and max(abs(a - b) for a, b in zip(cur, start)) > 0.02:
                moved = True
            if moved:
                if max(abs(a - b) for a, b in zip(cur, last)) < 0.002:
                    still_since = still_since or time.monotonic()
                    if time.monotonic() - still_since > 1.5:
                        return True
                else:
                    still_since = None
            last = cur
        return False

    # ------------------------------------------------------------ 서비스
    def run_exclusive(self, res, fn):
        if not self.busy.acquire(blocking=False):
            res.success, res.message = False, "다른 팔 동작 실행 중"
            return res
        try:
            res.success, res.message = fn()
        except Exception as e:  # noqa: BLE001  — 서비스가 죽지 않게
            res.success, res.message = False, f"예외: {e}"
        finally:
            self.busy.release()
        (self.get_logger().info if res.success else self.get_logger().error)(res.message)
        return res

    def srv_look(self, _req, res):
        return self.run_exclusive(res, lambda: self.move_joints(self.look_pose))

    def srv_stow(self, _req, res):
        return self.run_exclusive(res, lambda: self.move_joints(self.stow_pose))

    def srv_pick(self, _req, res):
        return self.run_exclusive(res, self.do_pick)

    def srv_drop(self, _req, res):
        return self.run_exclusive(res, self.do_drop)

    # ------------------------------------------------------------ 집기
    def stable_target(self, wait=3.0, n=5, spread=0.01):
        """최근 목표 n개가 spread(m) 안에 모이면 중앙값 반환."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            with self.lock:
                pts = [t[1:] for t in self.targets if time.monotonic() - t[0] < 1.5][-n:]
            if len(pts) >= n:
                med = [statistics.median(c) for c in zip(*pts)]
                if max(math.dist(p_, med) for p_ in pts) < spread:
                    return med
            time.sleep(0.1)
        return None

    def do_pick(self):
        target = self.stable_target()
        if target is None:
            return False, "안정된 YOLO 목표가 없음 (관측 자세에서 캔이 보이는지 확인)"
        msg = PointStamped()
        msg.header.frame_id = self.grasp_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point = Point(x=target[0], y=target[1], z=target[2])
        self.grasp_result = None
        self.grasp_target_pub.publish(msg)
        self.get_logger().info(f"grasp_node에 목표 전달: ({target[0]:.3f}, {target[1]:.3f}, "
                               f"{target[2]:.3f}) [{self.grasp_frame}]")
        t0 = time.monotonic()
        if self.grasp_result_topic:
            while self.grasp_result is None and time.monotonic() - t0 < self.pick_timeout:
                time.sleep(0.1)
            if self.grasp_result is None:
                return False, "grasp_node 결과 대기 시간 초과"
            if not self.grasp_result:
                return False, "grasp_node가 실패를 보고"
        elif not self.arm_settled_after_motion(self.pick_timeout):
            return False, "팔 동작 완료를 확인하지 못함"
        with self.lock:
            g = self.joints.get(self.gripper_joint)
        if g is not None and g < self.empty_close:
            return False, f"그리퍼가 끝까지 닫힘 ({g:.4f}) → 빈손으로 판단"
        return True, "집기 완료"

    # ------------------------------------------------------------ 놓기
    def do_drop(self):
        with self.lock:
            b = self.bin
        if b is None or time.monotonic() - b[0] > self.bin_memory:
            return False, "최근에 본 쓰레기통 마커(bin_center)가 없음 — 정렬(align)을 먼저 실행"
        bin_msg = b[1]
        age = time.monotonic() - b[0]
        if age > 1.0:
            self.get_logger().info(f"마커가 {age:.0f} s 전에 본 통 위치(odom 기준)를 사용")
        try:
            c_floor = self.tf.transform(bin_msg, self.floor_frame, timeout=Duration(seconds=0.2))
            bx, by = c_floor.pose.position.x, c_floor.pose.position.y
            pts = []
            for h in (self.drop_height + self.drop_lift, self.drop_height):
                ps = PointStamped()
                ps.header.frame_id = self.floor_frame
                ps.point = Point(x=bx, y=by, z=h)
                p1 = self.tf.transform(ps, "link1", timeout=Duration(seconds=0.2)).point
                pts.append((p1.x, p1.y, p1.z))
        except TransformException as e:
            return False, f"TF 변환 실패: {e}"

        plan = None
        for deg in self.drop_pitches:
            qs = [ik(*pt, math.radians(deg)) for pt in pts]
            if all(q is not None for q in qs):
                plan = (deg, qs)
                break
        if plan is None:
            r = math.hypot(pts[1][0], pts[1][1])
            reach = max_reach(pts[1][2])
            return False, (f"통 중심이 팔 범위 밖: link1에서 수평 {r:.3f} m, 최대 약 {reach:.3f} m "
                           f"→ bin_aligner standoff 를 {max(0.0, r - reach + 0.02):.3f} m 이상 줄이세요")
        deg, (q_above, q_drop) = plan
        self.get_logger().info(f"놓기: 통 중심 ({bx:.3f}, {by:.3f}) [{self.floor_frame}], pitch {deg:.0f} deg")

        self.bin_object(CollisionObject.ADD, bx, by)
        try:
            for q, what in [(q_above, "통 위로 이동"), (q_drop, "놓는 높이로 내림")]:
                ok, msg = self.move_joints(q)
                if not ok:
                    return False, f"{what} 실패: {msg}"
            if not self.gripper(self.gripper_open):
                return False, "그리퍼 열기 실패"
            time.sleep(0.5)
            ok, msg = self.move_joints(q_above)
            if not ok:
                return False, f"복귀 실패: {msg}"
        finally:
            self.bin_object(CollisionObject.REMOVE)
        return True, "놓기 완료"


def main():
    rclpy.init()
    node = ArmServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
