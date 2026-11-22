"""ArUco 4개 마커 인식 노드 (RPi에서 실행, OpenCV 4.5 / 4.7+ 모두 지원)

  python3 aruco_node.py
  -> /aruco/<name>_pose   마커 중심 '표면' 위치·방향 (PoseStamped, 카메라 광학 프레임)
  -> /aruco/<name>_center 원통 중심축 위치 (cyl_radius 를 준 마커만)

MARKERS 의 size 는 인쇄물을 자로 잰 '검은 테두리 바깥 끝~끝' 길이(m).
cyl_radius 는 원형 쓰레기통 반지름(m) = 둘레 / (2*pi). 평평한 벽은 넣지 않는다.
  (원통에는 마커를 똑바로 세워, 가로 방향으로 감기도록 붙일 것)
"""
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

# ---- 여기만 수정 -------------------------------------------------------
MARKERS = {
    0: dict(name="bin", size=0.060, cyl_radius=0.095),    # 원형 쓰레기통 (지름 19 cm)
    1: dict(name="wall_1", size=0.080),                    # 평평한 벽
    2: dict(name="wall_2", size=0.080),
    3: dict(name="wall_3", size=0.080),
}
OFFSET = {}          # 실측 보정값 (m, 카메라 광학 프레임) 예: {0: (0.0, 0.0, 0.004)}
AVG_N = 5            # 이동 평균 프레임 수
IMAGE_TOPIC = "/camera/image_raw"
INFO_TOPIC = "/camera/camera_info"
# ----------------------------------------------------------------------


def object_points(cfg):
    """마커 네 모서리의 3D 좌표 (좌상, 우상, 우하, 좌하). 곡면이면 현(chord) 폭 사용."""
    s = cfg["size"]
    r = cfg.get("cyl_radius")
    if r:
        theta = s / r                       # 감긴 중심각
        w = 2 * r * np.sin(theta / 2)       # 네 모서리가 이루는 평면의 실제 폭
    else:
        w = s
    h = s
    return np.array([[-w / 2, h / 2, 0], [w / 2, h / 2, 0],
                     [w / 2, -h / 2, 0], [-w / 2, -h / 2, 0]], np.float32)


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")
        self.bridge = CvBridge()
        self.K = self.D = None
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):                 # OpenCV >= 4.7
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            det = cv2.aruco.ArucoDetector(d, p)
            self.detect = det.detectMarkers
        else:                                                    # OpenCV 4.5.x
            p = cv2.aruco.DetectorParameters_create()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detect = lambda g: cv2.aruco.detectMarkers(g, d, parameters=p)

        self.obj = {i: object_points(c) for i, c in MARKERS.items()}
        self.hist = {i: deque(maxlen=AVG_N) for i in MARKERS}
        self.pub = {i: self.create_publisher(PoseStamped, f"/aruco/{c['name']}_pose", 10)
                    for i, c in MARKERS.items()}
        self.pub_center = {i: self.create_publisher(PoseStamped, f"/aruco/{c['name']}_center", 10)
                           for i, c in MARKERS.items() if c.get("cyl_radius")}
        self.create_subscription(CameraInfo, INFO_TOPIC, self.info_cb, 10)
        self.create_subscription(Image, IMAGE_TOPIC, self.img_cb, 10)
        self.get_logger().info(f"markers: { {i: c['name'] for i, c in MARKERS.items()} }")

    def info_cb(self, m):
        self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(m.d, dtype=np.float64)

    def img_cb(self, msg):
        if self.K is None:
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect(gray)
        seen = set()
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                i = int(i)
                if i not in MARKERS:
                    continue
                ok, rvec, tvec = cv2.solvePnP(self.obj[i], c.reshape(4, 2).astype(np.float64),
                                              self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
                if not ok:
                    continue
                rot = R.from_rotvec(rvec.flatten())
                # 모서리 평면 중심(평면 마커면 그대로 표면 중심)
                pos = tvec.flatten() + np.array(OFFSET.get(i, (0, 0, 0)))
                self.hist[i].append(pos)
                seen.add(i)
                mean = np.mean(self.hist[i], axis=0)
                normal = rot.as_matrix()[:, 2]              # 마커 밖(카메라 쪽) 방향
                r = MARKERS[i].get("cyl_radius")
                q = rot.as_quat()
                if r:
                    # 모서리 평면은 표면보다 sagitta 만큼 안쪽 -> 표면 중심으로 보정
                    half = MARKERS[i]["size"] / r / 2
                    surface = mean + normal * r * (1 - np.cos(half))
                    center = mean - normal * r * np.cos(half)  # 원통 중심축
                    self.publish(self.pub[i], msg.header, surface, q)
                    self.publish(self.pub_center[i], msg.header, center, q)
                else:
                    self.publish(self.pub[i], msg.header, mean, q)
        for i in MARKERS:                 # 안 보이면 평균 초기화 (오래된 값 재사용 방지)
            if i not in seen:
                self.hist[i].clear()

    @staticmethod
    def publish(pub, header, pos, q):
        p = PoseStamped()
        p.header = header
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, pos)
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = map(float, q)
        pub.publish(p)


def main():
    rclpy.init()
    rclpy.spin(ArucoNode())


if __name__ == "__main__":
    main()
