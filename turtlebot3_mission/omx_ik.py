"""OpenMANIPULATOR-X 해석적 역기구학 (link1 기준, ROS 의존성 없음)

치수는 OpenMANIPULATOR-X 공식 URDF 기준. turtlebot3_manipulation 의 URDF와 다르면 DIMS 수정.
  joint1 원점: link1에서 (0.012, 0, 0.017)
  joint2 원점: joint1에서 (0, 0, 0.0595)
  joint3 원점: joint2에서 (0.024, 0, 0.128)
  joint4 원점: joint3에서 (0.124, 0, 0)
  그리퍼 중심(end_effector_link): joint4에서 (0.126, 0, 0)
관절 2~4는 +y축 회전 → 양수일수록 앞으로 숙인다.
pitch: 그리퍼가 수평에서 아래로 숙인 각도 (rad, 아래가 +)
"""
import math

DIMS = dict(j1_x=0.012, j2_z=0.017 + 0.0595, l2_x=0.024, l2_z=0.128, l3=0.124, l4=0.126)
LIMITS = [(-math.pi * 0.9, math.pi * 0.9), (-1.5, 1.5), (-1.5, 1.4), (-1.7, 1.97)]


def fk(q, d=DIMS):
    """관절각 → 그리퍼 중심 위치 (link1 기준) 와 pitch."""
    q1, q2, q3, q4 = q
    a1 = math.atan2(d["l2_z"], d["l2_x"]) - q2      # link2 수평 기준 앙각
    a2 = -(q2 + q3)                                  # link3 앙각
    a3 = -(q2 + q3 + q4)                             # 그리퍼 앙각
    L1 = math.hypot(d["l2_x"], d["l2_z"])
    r = L1 * math.cos(a1) + d["l3"] * math.cos(a2) + d["l4"] * math.cos(a3)
    z = L1 * math.sin(a1) + d["l3"] * math.sin(a2) + d["l4"] * math.sin(a3)
    return (d["j1_x"] + r * math.cos(q1), r * math.sin(q1), d["j2_z"] + z), -a3


def ik(x, y, z, pitch, d=DIMS, limits=LIMITS):
    """그리퍼 중심 (x, y, z) [link1 기준] 과 pitch → 관절각 [q1..q4] (elbow-up). 불가능하면 None."""
    dx = x - d["j1_x"]
    q1 = math.atan2(y, dx)
    r = math.hypot(dx, y)
    zr = z - d["j2_z"]
    rw = r - d["l4"] * math.cos(pitch)              # 손목(joint4) 위치
    zw = zr + d["l4"] * math.sin(pitch)
    L1, L2 = math.hypot(d["l2_x"], d["l2_z"]), d["l3"]
    c = (rw * rw + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1.0:
        return None
    delta = -math.acos(c)                            # elbow-up: link3가 link2보다 아래로 꺾임
    a1 = math.atan2(zw, rw) - math.atan2(L2 * math.sin(delta), L1 + L2 * math.cos(delta))
    a2 = a1 + delta
    q2 = math.atan2(d["l2_z"], d["l2_x"]) - a1
    q3 = -a2 - q2
    q4 = pitch - q2 - q3
    q = [q1, q2, q3, q4]
    if any(not (lo <= v <= hi) for v, (lo, hi) in zip(q, limits)):
        return None
    return q


def max_reach(z, pitch=0.0, d=DIMS):
    """높이 z(link1 기준)에서 pitch로 닿는 최대 수평 거리 (joint1 축 기준, 근사)."""
    lo, hi = 0.0, 0.6
    for _ in range(40):
        mid = (lo + hi) / 2
        if ik(d["j1_x"] + mid, 0.0, z, pitch, d) is not None:
            lo = mid
        else:
            hi = mid
    return lo
