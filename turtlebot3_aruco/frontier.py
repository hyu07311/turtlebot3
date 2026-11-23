"""SLAM 지도(OccupancyGrid)에서 탐색할 경계(frontier) 찾기 — 순수 numpy (ROS 없이 테스트 가능)

frontier = '비어 있다고 확인된 칸' 중에서 '아직 모르는 칸'과 맞닿은 칸.
frontier 가 더 이상 없으면 (= 갈 수 있는 곳은 다 봤으면) 맵핑 완료.
ArUco 마커(맵의 끝) 뒤쪽의 frontier 는 탐색하지 않는다.
"""
import math

import numpy as np
from scipy import ndimage

FREE_MAX = 25        # 이 값 이하 = 빈 칸 (slam_toolbox: 0 빈 칸, 100 장애물, -1 모름)
OCC_MIN = 65


def frontier_clusters(data, width, height, res, ox, oy, min_size_m=0.30, clear_m=0.15):
    """frontier 덩어리 목록 [(x, y, 크기 m, 칸 수), ...] (지도 좌표, 덩어리 중심에 가장 가까운 frontier 칸).

    clear_m: 장애물에서 이만큼 떨어진 빈 칸만 frontier 로 본다 (벽 틈 · 라이다 잡음 무시).
    """
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    free = (g >= 0) & (g <= FREE_MAX)
    unknown = g < 0
    occ = g >= OCC_MIN
    if clear_m > 0 and occ.any():
        r = max(1, int(round(clear_m / res)))
        near_occ = ndimage.binary_dilation(occ, iterations=r)
        free_ok = free & ~near_occ
    else:
        free_ok = free
    unk_nb = ndimage.binary_dilation(unknown, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool))
    front = free_ok & unk_nb
    labels, n = ndimage.label(front, structure=np.ones((3, 3), bool))
    out = []
    min_cells = max(2, int(min_size_m / res))
    for k in range(1, n + 1):
        ys, xs = np.nonzero(labels == k)
        if len(xs) < min_cells:
            continue
        cx, cy = xs.mean(), ys.mean()
        j = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
        out.append((ox + (xs[j] + 0.5) * res, oy + (ys[j] + 0.5) * res, len(xs) * res, len(xs)))
    return out


def behind_marker(x, y, marker, depth=0.25, radius=4.0, half_width=1.0):
    """(x, y) 가 맵의 끝 마커 '바로 뒤쪽 띠' 안인가: 마커 면에서 depth 이내 앞쪽부터 뒤로 radius(4 m) 까지,
    좌우로 half_width(1 m) 이내. (방 안 원통 쓰레기통처럼 아무 방향을 향한 마커가 방 전체를 막지 않게 좁은 띠로)
    marker = (mx, my, nx, ny): 마커 위치와 법선(탐색한 쪽을 향함)."""
    mx, my, nx, ny = marker
    dx, dy = x - mx, y - my
    return (dx * nx + dy * ny < depth and abs(dx * ny - dy * nx) < half_width
            and math.hypot(dx, dy) < radius)


def choose(clusters, robot_xy, ends=(), blacklist=(), black_r=0.4, near=0.45, push=0.5):
    """갈 frontier 선택: 마커 뒤 · 실패 목록 제외, (거리 - 크기 보너스)가 가장 작은 것.
    로봇 바로 옆(near 이내)의 경계는 그쪽 방향으로 push 만큼 더 들어간 지점을 목표로 한다 (모르는 쪽으로 한 걸음).
    반환: (선택 (x, y, 크기, 칸 수) 또는 None, 유효 덩어리 수)."""
    rx, ry = robot_xy
    valid = []
    for c in clusters:
        x, y, size, n = c
        if any(behind_marker(x, y, m) for m in ends):
            continue
        if any(math.hypot(x - bx, y - by) < black_r for bx, by in blacklist):
            continue
        d = math.hypot(x - rx, y - ry)
        if d < near:
            k = (d + push) / max(d, 1e-3)
            x, y = rx + (x - rx) * k, ry + (y - ry) * k
        valid.append((d - 0.5 * min(size, 2.0), (x, y, size, n)))
    if not valid:
        return None, 0
    valid.sort(key=lambda v: v[0])
    return valid[0][1], len(valid)


def viewpoints(data, width, height, res, ox, oy, spacing=1.2, clear=0.40):
    """마커 찾기용 관측 지점: 장애물에서 clear 이상 떨어진 빈 칸을 spacing 격자로 고른 목록 [(x, y), ...]."""
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    free = (g >= 0) & (g <= FREE_MAX)
    blocked = (g >= OCC_MIN) | (g < 0)
    r = max(1, int(round(clear / res)))
    ok = free & ~ndimage.binary_dilation(blocked, iterations=r)
    step = max(1, int(round(spacing / res)))
    pts = []
    for y0 in range(0, height, step):
        for x0 in range(0, width, step):
            ys, xs = np.nonzero(ok[y0:y0 + step, x0:x0 + step])
            if len(xs):
                cx, cy = xs.mean(), ys.mean()            # 칸 안에서 중심에 가장 가까운 빈 칸
                j = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
                pts.append((ox + (x0 + xs[j] + 0.5) * res, oy + (y0 + ys[j] + 0.5) * res))
    return pts
