"""캔 거치대 앞 정렬 노드 (Orin에서 실행) — bin_aligner 알고리즘 재사용, 마커만 ID 3(/aruco/stand_pose)

Nav2로 TRASH(거치대) 지점에 도착한 뒤, Pi 카메라로 본 거치대 마커(ID 3)를 기준으로
'마커 정면 standoff' 에 정렬하고, 캔 중심이 팔(link1) 앞 target_reach 에 오도록 오도메트리로 직진한다.
(집기는 캔이 link1 앞 약 0.29~0.33 m 에 있어야 함. Nav2 도착 오차 5~10 cm 를 여기서 줄인다)

  python3 stand_aligner.py
  ros2 service call /stand_aligner/align std_srvs/srv/Trigger

직진 거리 advance = standoff - (link1_x + target_reach - can_offset)
  standoff     : base_link 원점 ~ 마커 면 정렬 거리 (Pi 카메라가 마커를 선명하게 보는 거리, 기본 0.25)
  can_offset   : 마커 면 ~ 캔 중심 (마커 법선 방향, 거치대에 붙인 위치에 따라 실측) 기본 0.05
  target_reach : link1 ~ 캔 중심 목표 수평 거리 (pick&place 실측 성공 범위 0.29~0.33) 기본 0.30
  link1_x      : base_link ~ link1 (URDF -0.092)
advance 를 직접 주면(>=0) 그 값을 쓴다.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

from bin_aligner import BinAligner

DEFAULTS = {
    "pose_topic": "/aruco/stand_pose",
    "service": "/stand_aligner/align",
    "standoff": 0.25,
    "advance": -1.0,          # -1 = 아래 식으로 자동 계산
    "total_timeout": 70.0,    # Nav2 도착 오차가 커도(±10 cm) 재보정할 시간
    "max_retries": 5,         # 좌우 재보정(후진 후 재접근) 최대 횟수
}


class StandAligner(BinAligner):
    def __init__(self):
        super().__init__("stand_aligner", DEFAULTS)
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.can_offset = p("can_offset", 0.05)
        self.target_reach = p("target_reach", 0.30)
        self.link1_x = p("link1_x", -0.092)
        if self.advance < 0:
            self.advance = self.standoff - (self.link1_x + self.target_reach - self.can_offset)
        if self.advance < 0:
            self.get_logger().warn(f"계산된 직진 거리가 음수({self.advance:.3f}) → 0 (standoff 를 늘릴 것)")
            self.advance = 0.0
        self.get_logger().info(
            f"거치대 정렬: standoff {self.standoff:.3f} m → 직진 {self.advance:.3f} m "
            f"→ 캔 중심이 link1 앞 {self.target_reach:.2f} m (can_offset {self.can_offset:.3f})")


def main():
    rclpy.init()
    node = StandAligner()
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
