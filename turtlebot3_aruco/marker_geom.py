"""ArUco 마커 ↔ 지도 좌표 계산 (ROS 없이 테스트 가능한 순수 함수)

좌표는 모두 2D (x, y, yaw). 마커의 yaw 는 '마커 법선(마커 밖, 보는 사람 쪽) 방향'.
"""
import math
import statistics


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_z_axis(q):
    """쿼터니언 회전의 z축 벡터 (= ArUco 마커 법선, 마커 밖·카메라 쪽 방향)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """a ⊕ b : a 프레임에서 본 b 를 a 의 부모 프레임으로."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(at + bt))


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def robot_from_marker(marker_map, marker_base):
    """지도상의 마커 포즈와 로봇(base)에서 본 마커 포즈 → 지도상의 로봇 포즈."""
    return compose(marker_map, inverse(marker_base))


def robot_from_point(point_map, point_base, yaw):
    """방향을 모르는 점(원통 중심축 등) 하나 + 로봇 yaw → 지도상의 로봇 위치."""
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = point_base
    return (point_map[0] - (c * px - s * py), point_map[1] - (s * px + c * py), yaw)


def median_pose(poses):
    """(x, y, yaw) 목록의 중앙값과 퍼짐(표준편차: m, rad)."""
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    ref = poses[len(poses) // 2][2]
    ts = [ref + wrap(p[2] - ref) for p in poses]          # 각도 경계(±180°) 넘김 처리
    med = (statistics.median(xs), statistics.median(ys), wrap(statistics.median(ts)))
    if len(poses) < 2:
        return med, (0.0, 0.0)
    pos_sd = math.hypot(statistics.pstdev(xs), statistics.pstdev(ys))
    return med, (pos_sd, statistics.pstdev(ts))


def approach_point(mx, my, myaw, dist):
    """마커 법선 방향으로 dist 앞, 마커를 정면으로 바라보는 로봇 포즈 (Nav2 웨이포인트)."""
    return (mx + dist * math.cos(myaw), my + dist * math.sin(myaw), wrap(myaw + math.pi))
