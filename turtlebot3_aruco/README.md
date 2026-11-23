# TurtleBot3 ArUco 마커 인식 · 쓰레기통 정렬

TurtleBot3 Waffle Pi + OpenMANIPULATOR-X(RM-X52-TNM) 쓰레기 수거 시연에서 사용하는 ArUco 마커 인식,
쓰레기통 앞 정렬 코드와 인쇄물.

- **시나리오**: 출발 → Nav2 이동 → 쓰레기 줍기(D435i + YOLO) → Nav2 이동 → **쓰레기통 마커로 정렬** → 버리기
- **카메라 역할**
  - D435i: **OMX 팔에 장착** (eye-in-hand) → YOLO 인식 · 뎁스. 집는 순간 시야가 가려짐
  - Pi 카메라: **로봇 몸체에 고정** → ArUco 마커 인식 (쓰레기통 · 주요 포인트 · 캔 거치대)
- 보드 구성·역할은 [저장소 README](../README.md) 기준. 라즈베리파이는 **Pi 카메라 + ArUco 인식만** 담당하고,
  OpenCR(바퀴·팔)·LDS-03·D435i는 모두 Orin에 연결된다. 이 문서는 ArUco 관련 부분만 다룬다.

---

## 1. 시스템 구성

```
[스마트폰 핫스팟] -- Wi-Fi -- 라파4, Orin Nano, (내 PC)   : 인터넷, SSH
라파4 (192.168.10.1) == 유선랜 직결 == Orin Nano (192.168.10.2) : ROS 2 통신 전용 (Cyclone DDS)

[라파4]                                          [Orin Nano]
 camera_ros: Pi 카메라                            OpenCR(USB): 바퀴 · 팔 (turtlebot3_manipulation, /odom, /cmd_vel)
   └▶ aruco_node.py ─── /aruco/* ──────────────▶  bin_aligner.py / stand_aligner.py ── /cmd_vel ──▶ 바퀴
                                                  LDS-03(USB): SLAM · Nav2
                                                  D435i(팔 손목) + YOLO, MoveIt, 시나리오
```

| 항목 | 내용 |
|---|---|
| ROS | ROS 2 Humble, `ROS_DOMAIN_ID=30`, `rmw_cyclonedds_cpp` (유선 인터페이스 고정) |
| Pi 카메라 | Raspberry Pi Camera v2.1 (IMX219), 640×480, 라파4에서 발행 |
| ArUco 처리 | **라파** (라파의 역할은 Pi 카메라 + ArUco 인식뿐. 결과 `/aruco/*` 만 Orin으로 보냄) |
| 마커 사전 | `DICT_4X4_50` |

---

## 2. 파일 구조

```
.
├── aruco_node.py        # 마커 인식 (라파, Pi 카메라 영상 직접 구독 USE_COMPRESSED=False)
├── bin_aligner.py       # 쓰레기통 앞 미세 정렬 (Orin, /cmd_vel)
├── stand_aligner.py     # 캔 거치대(ID 3) 앞 정렬 (Orin, bin_aligner 알고리즘 재사용)
├── aruco_explore.py     # SLAM 자율 탐색: frontier → Nav2, 지도 완성 시 자동 저장 · 종료, 마커 = 맵의 끝 (Orin)
├── frontier.py          # 지도에서 탐색 경계(frontier) · 마커 뒤쪽 제외 · 관측 지점 계산
├── marker_mapper.py     # SLAM 맵핑 중 마커 지도 좌표 기록 + waypoints.yaml 자동 생성 (Orin)
├── aruco_localizer.py   # Nav2 주행 중 마커로 AMCL 위치 보정 (/initialpose, Orin)
├── marker_geom.py       # 위 두 노드의 좌표 계산 (순수 함수)
├── make_markers.py      # 인쇄용 마커 PDF 생성
└── markers/
    ├── aruco_markers_60mm.pdf   # ID 0 (쓰레기통)       ← 인쇄
    ├── aruco_markers_80mm.pdf   # ID 1, 2 (주요 포인트), 3 (캔 거치대) ← 인쇄
    └── *.png                    # 미리보기
```

---

## 3. 마커 구성

| ID | 부착 위치 | 크기 | 비고 | 발행 토픽 |
|---|---|---|---|---|
| 0 | 원형 쓰레기통 (지름 19 cm, 높이 20 cm) | 60 mm | 원통 곡면 보정 | `/aruco/bin_pose`, `/aruco/bin_center` |
| 1 | 주요 포인트 | 80 mm | 평면, 이동 중 위치 보정 | `/aruco/wall_1_pose` |
| 2 | 주요 포인트 | 80 mm | 평면, 이동 중 위치 보정 | `/aruco/wall_2_pose` |
| 3 | 콜라캔 거치대 | 80 mm | 평면, 거치대 접근 보정 | `/aruco/stand_pose` |

| 쓰레기통 (60 mm) | 벽 (80 mm) |
|---|---|
| <img src="markers/aruco_markers_60mm_p1.png" width="260"> | <img src="markers/aruco_markers_80mm_p1.png" width="260"> |

### 크기 선정 근거
Pi Camera v2 (640×480, fx ≈ 532 px)에서 마커가 약 30 px 이상이면 안정적으로 인식된다.

| 크기 | 0.5 m | 1.0 m | 1.5 m |
|---|---|---|---|
| 40 mm | 43 px | 21 px ✗ | 14 px ✗ |
| 60 mm | 64 px | 32 px | 21 px |
| 80 mm | 85 px | 43 px | 28 px |

쓰레기통은 지름이 작아(19 cm) 80 mm를 붙이면 둘레의 48°를 감싸 테두리가 크게 휜다 → **60 mm** (36°, 중앙 돌출 4.7 mm).

---

## 4. 인쇄 · 부착

1. `markers/aruco_markers_60mm.pdf` (1장), `markers/aruco_markers_80mm.pdf` (2장) 인쇄
2. 인쇄 배율 **"실제 크기(100%)"** — "페이지에 맞춤" 끄기
3. 상단 기준선이 **정확히 50 mm** 인지 자로 확인
4. 회색 **점선을 따라** 자르기 (안쪽 흰 여백은 인식에 필수)
5. 검은 테두리 바깥 끝~끝을 실측해서 `aruco_node.py`의 `size`에 입력
6. 부착
   - 쓰레기통: 마커를 **똑바로 세워서** 가로 방향으로 휘도록 밀착 (90° 돌리면 보정 방향이 틀어짐)
   - 쓰레기통 · 벽 모두 **Pi 카메라와 비슷한 높이** (몸체 고정 카메라라 위아래로 못 봄)
   - 무광 종이 사용, 테이프로 마커 면을 덮지 않기

다른 크기·ID가 필요하면:
```bash
pip install opencv-python pillow
python make_markers.py 80 4 5      # 80 mm, ID 4·5
```

---

## 5. 환경 설정

네트워크(유선 고정 IP), Cyclone DDS, chrony, `.bashrc`는 **팀 가이드 4~6장** 그대로 따른다.
ArUco 작업에 추가로 필요한 것만 정리한다.

### 5.1 라파4: Pi 카메라
```bash
sudo apt install -y ros-humble-camera-ros ros-humble-image-transport-plugins
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 \
  -p camera_info_url:=file:///home/turtlebot/pi_camera.yaml
```
- 라파 사용자 이름은 `turtlebot` (홈: `/home/turtlebot`).
- 인식은 라파에서 raw 영상으로 하므로 압축 영상은 필수가 아니다. Orin에서 화면을 볼 때만 `image-transport-plugins` 로 `/camera/image_raw/compressed` 를 쓴다.
- 카메라 미인식 시 `/boot/firmware/config.txt` 에 `dtoverlay=imx219` 추가 후 재부팅.
- 최신 터틀봇3는 Pi 카메라를 도커로 지원하는 변경이 있으니 e-Manual도 확인 (팀 가이드 5.7).

### 5.2 패키지 · 통신
```bash
# 라파 (aruco_node) — 2026-11-22 설치 완료
sudo apt install -y ros-humble-camera-ros ros-humble-cv-bridge ros-humble-image-transport-plugins python3-opencv python3-scipy
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
# 라파 유선 고정 IP: /etc/netplan/60-ros-direct.yaml (eth0: 192.168.10.1/24, 게이트웨이 없음 → 인터넷은 Wi-Fi)
# 라파 ~/.bashrc: ROS_DOMAIN_ID=30, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, CYCLONEDDS_URI=~/.ros/cyclonedds.xml (eth0 고정)
# Orin (bin_aligner)
sudo apt install -y ros-humble-tf2-geometry-msgs
```

### 5.2.1 라파 카메라 시작 스크립트 · 보정 파일 (2026-11-23)
- [`../pi_tools/start_camera.sh`](../pi_tools/start_camera.sh) → 라파 `~/turtlebot/pi_tools/` : `-p sensor_mode:=1640:1232`(센서 **전체 화각**) + 640×480 RGB888 출력, `~/pi_camera.yaml` 자동 적용
  - `sensor_mode` 를 안 주면 640×480 모드가 **센서 가운데만 잘라 써서 화각이 약 23°** 로 좁아진다 (전체 화각은 약 62°)
- `frame_id:=camera_rgb_optical_frame` (로봇 URDF 의 Pi 카메라 광학 프레임) — 기본값 `camera` 는 TF 트리에 없어 `bin_aligner` 가 TF 변환에 실패함
- [`../pi_tools/pi_camera.yaml`](../pi_tools/pi_camera.yaml) → 라파 `~/pi_camera.yaml` : fx = fy = 525, 주점 (320, 240), k1 0.213, k2 −0.389
  - 체커보드 내부 7×10, 25 mm. 손으로 든 사진만으로는 초점거리가 정해지지 않아(정면 위주 → 거리와 화각 구분 불가) **50 cm 정면 사진으로 fx 실측**(517~533) 후 고정, 왜곡만 49장으로 계산 (재투영 오차 1.7 px — 판에 고정한 보드로 다시 하면 개선 가능)

### 5.3 Pi 카메라 캘리브레이션 (필수)
캘리브레이션 값(`camera_info`)이 없으면 `aruco_node`는 아무것도 발행하지 않는다.
```bash
# Orin(모니터 연결)에서 라파 카메라 영상을 받아 진행. 체커보드 8x6 내부코너, 한 칸 25 mm 예시
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -p camera:=/camera
```
보드를 여러 위치·각도·거리로 움직여 X/Y/Size/Skew 바가 초록색이 되면 **CALIBRATE → SAVE** →
yaml을 라파로 복사(`~/pi_camera.yaml`)해서 `camera_info_url` 로 지정.

### 5.4 Pi 카메라 TF
몸체 고정이므로 `base_link → 카메라 광학 프레임` 은 고정값이다.
- 로봇 URDF(Orin의 `turtlebot3_manipulation` robot_state_publisher)에 Pi 카메라용 `camera_link` 계열 프레임이 이미 있다.
  카메라를 원래 자리에 그대로 두었다면 camera_ros의 `header.frame_id` 를 이 이름과 맞추면 된다.
- 위치를 옮겼다면 실측해서 static TF를 추가:
  ```bash
  ros2 run tf2_ros static_transform_publisher --x .. --y .. --z .. \
    --roll -1.5708 --pitch 0 --yaw -1.5708 \
    --frame-id base_link --child-frame-id <camera_ros frame_id>
  ```
  (광학 프레임은 z가 앞, 로봇은 x가 앞이라 위 회전이 기본. 카메라가 기울어 있으면 그만큼 더함)
- 확인: `ros2 run tf2_ros tf2_echo base_link <camera frame>`

---

## 6. 실행

```bash
# 라파: Pi 카메라 + 마커 인식 (라파는 이것만 실행) — 스크립트는 ../pi_tools 를 라파 ~/turtlebot/pi_tools 에 둠
~/turtlebot/pi_tools/start_camera.sh   # 전체 화각 1640x1232 → 640x480, ~/pi_camera.yaml 적용
~/turtlebot/pi_tools/start_aruco.sh    # aruco_node.py (라파용 기본값 USE_COMPRESSED = False), 로그 /tmp/pi_aruco.log

# Orin: 확인
ros2 topic hz /aruco/bin_pose          # 10 Hz 이상
ros2 topic echo /aruco/wall_1_pose     # 30 cm 앞에 두면 position.z ≈ 0.30

# Orin: 바퀴·팔 bringup 이 떠 있는 상태에서 (pick&place 의 system.launch.py 또는 turtlebot3_manipulation base.launch.py)
# 쓰레기통 정렬 (Nav2 도착 후)
python3 bin_aligner.py --ros-args -p advance:=0.12   # 직진 거리는 팔 마운트에 맞게
ros2 service call /bin_aligner/align std_srvs/srv/Trigger

# 캔 거치대 정렬 (Nav2 로 거치대 지점 도착 후) — 캔이 link1 앞 약 0.30 m 에 오도록 자동 직진
python3 stand_aligner.py --ros-args -p can_offset:=0.05   # 마커 면 ~ 캔 중심 거리 (거치대에서 실측)
ros2 service call /stand_aligner/align std_srvs/srv/Trigger
```

마커로 위치 보정 (맵핑 → 주행) — 실행 방법은 [slam_nav2 README 1-1](https://github.com/hanjaejin124/turtlebot/blob/main/slam_nav2/README.md) 참고
```bash
~/slam_nav2/slam_aruco.sh        # 출발점에서 실행 → w: 자율 탐색, 지도가 다 그려지면 자동 저장 (마커 = 맵의 끝)
~/slam_nav2/nav2_aruco.sh        # Nav2 + aruco_localizer
ros2 service call /aruco_localizer/correct std_srvs/srv/Trigger   # 지금 바로 보정
```
`aruco_localizer`: 평면 마커(ID 1·2·3)는 x, y, yaw 모두 계산하고, yaw 는 AMCL 과 반반 섞는다(`yaw_weight` 0.5).
원통 쓰레기통(ID 0)은 중심축 위치만 써서 x, y 만 보정. 멈춰 있을 때 1초간 8프레임 이상, 흔들림 2 cm·3° 이하일 때만,
차이 3 cm/2° 이상 1 m/30° 이하일 때 보정. 가짜 TF·마커(잡음 2 mm, 1°)로 확인: 마커 지도 좌표 오차 0.3 cm 이내,
보정 결과가 손 계산과 1 cm 이내.

`stand_aligner` 파라미터 (기본값): `standoff` 0.25 m (마커 정면 정렬 거리), `can_offset` 0.05 m,
`target_reach` 0.30 m (pick&place 실측 성공 범위 0.29~0.33 m), `link1_x` -0.092 m, `advance` -1 (= 자동 계산
`standoff - (link1_x + target_reach - can_offset)`), `total_timeout` 70 s, `max_retries` 5.
시뮬레이션(실제 제어 코드 + 차동구동 모델): Nav2 도착 오차 ±5 cm → 39/40, ±10 cm → 36/40 이 집기 범위 안. 실기 테스트 전.
Orin에서 압축 영상을 받아 인식하는 방식(`USE_COMPRESSED = True`)도 코드에 남아 있지만, 현재 구성은 라파에서 인식한다.

---

## 7. `aruco_node.py` — 마커 인식

### 설정 (이 부분만 수정)
```python
MARKERS = {
    0: dict(name="bin",    size=0.060, cyl_radius=0.095),  # 원형 쓰레기통 (지름 19 cm)
    1: dict(name="wall_1", size=0.080),
    2: dict(name="wall_2", size=0.080),
    3: dict(name="stand",  size=0.080),                     # ID 3 = 콜라캔 거치대
}
OFFSET = {}            # 실측 보정값 (m, 카메라 광학 프레임) 예: {0: (0.0, 0.0, 0.004)}
AVG_N = 5              # 이동 평균 프레임 수
USE_COMPRESSED = False # 라파에서 실행: False (현재 구성, 기본값) / Orin에서 압축 영상 구독: True
```

| 함수 | 역할 |
|---|---|
| `object_points()` | 마커 네 모서리 3D 좌표 생성. 원통 마커는 현(chord) 폭 사용 |
| `info_cb()` | `camera_info`에서 내부 파라미터 K, 왜곡 D 수신 |
| `to_gray()` | 압축(JPEG) 또는 raw 영상을 흑백으로 변환 |
| `img_cb()` | 마커 검출 → `solvePnP(IPPE)` → 원통 보정 → 5프레임 평균 → 발행 |
| 출력 | `PoseStamped`, **카메라 광학 프레임** 기준 (z 앞, x 오른쪽, y 아래) |

### 원통(쓰레기통) 곡면 보정
ArUco는 마커가 평평하다고 가정한다. 원통에 감긴 마커는:
1. 네 모서리는 휘어도 **한 평면 위**에 있다 → 폭을 현의 길이 `2r·sin(θ/2)` (θ = size/r) 로 두고 `solvePnP`
2. 그 평면은 표면보다 `r·(1−cos(θ/2))` 안쪽 → 표면 위치 보정 (`/aruco/bin_pose`)
3. 그 평면에서 `r·cos(θ/2)` 더 안쪽이 원통 중심축 (`/aruco/bin_center`)

합성 투영 검증 (r = 9.5 cm, 60 mm 마커):

| 조건 | 보정 없음 | 보정 후 |
|---|---|---|
| 0.5 m, 정면 | 4.7 mm | 0.00 mm |
| 0.8 m, 20° | 4.7 mm | 0.00 mm |
| 1.0 m, −30° | 4.7 mm | 0.00 mm |

> 실제 영상에서는 모서리 검출 오차가 추가되므로 아래 실측 보정을 함께 한다.

### 실측 오프셋 보정
1. 카메라 앞 알려진 위치에 마커를 두고 실제 거리를 자로 측정
2. `ros2 topic echo`로 10회 정도 평균
3. 3~5개 위치(가까이/멀리, 좌/우)에서 반복
4. (실측 − 측정) 평균을 `OFFSET`에 입력
   위치마다 오차가 크게 다르면 `size`나 카메라 캘리브레이션 문제

---

## 8. `bin_aligner.py` — 쓰레기통 앞 정렬

Nav2 도착 오차(약 5 cm, 수 도)를 Pi 카메라로 본 쓰레기통 마커 기준으로 줄인다.
와플파이는 옆으로 못 움직이므로 **주차하듯 3단계**로 맞추고, 필요하면 마지막에 직진한다.

```
APPROACH ── 목표 거리 도달 ──┬── 좌우 OK ──▶ ROTATE ── 오차 0.5 s 유지 ──▶ (ADVANCE) ──▶ 완료
   ▲                        └── 좌우 남음 ─▶ BACKOFF (15 cm 후진하며 좌우 보정)
   └──────────────────────────────────────────┘          (최대 3회)
```

| 단계 | 동작 |
|---|---|
| APPROACH | 마커 법선 위 직선을 따라 목표 거리(`standoff`)까지 전진/후진. 좌우 오차에 비례해 최대 15° 기울여 직선으로 복귀 |
| BACKOFF | 목표 거리에 왔는데 좌우 오차가 남으면 `backoff` 만큼 후진하며 보정 |
| ROTATE | 제자리 회전으로 정면 각도 맞춤 → 허용 오차 이내 0.5 s 유지되면 정지 |
| ADVANCE | (`advance` > 0) 정렬한 방향을 유지하며 `/odom` 으로 `advance` 만큼 직진. 마커가 너무 가까워 안 보여도 됨 |
| 안전 | 마커 0.3 s 안 보이면 정지, 2 s 안 보이면 실패. 전체 40 s 초과 시 실패. 종료 시 항상 정지 명령 |

- 호출: `/bin_aligner/align` (`std_srvs/Trigger`) — 정렬이 끝날 때까지 블록하고 성공/실패와 최종 오차를 반환
- 단독 테스트: `python3 bin_aligner.py --ros-args -p auto_start:=true`

### 주요 파라미터
| 이름 | 기본값 | 의미 |
|---|---|---|
| `standoff` | 0.25 m | base_link 원점 ~ 통 표면 정렬 거리. Pi 카메라가 마커를 잘 보는 거리 |
| `advance` | 0.0 m | 정렬 후 직진 거리. **표준 팔 마운트면 0.12 m** (팔이 통 중심 위에 닿으려면 통 표면이 base_link에서 0.153 m 이내 — [`../turtlebot3_mission/README.md`](../turtlebot3_mission/README.md) 4장) |
| `pos_tol` / `lat_tol` | 15 mm | 앞뒤 / 좌우 허용 오차 (통 지름 19 cm 기준 충분한 여유) |
| `yaw_tol_deg` | 3° | 각도 허용 오차 |
| `max_v` / `max_w` | 0.05 m/s / 0.3 rad/s | 최대 속도 |
| `psi_max_deg` | 15° | 주행 중 최대 기울기 (마커를 시야 안에 유지) |
| `backoff` | 0.15 m | 좌우 재보정 후진 거리 |

### 시뮬레이션 검증
차동 구동 모델 + 5프레임 평균 측정으로 실제 `control_step()`을 돌림.
초기 오차 앞뒤 {−5, 0, +5} cm × 좌우 {−5, −2, 0, +2, +5} cm × 각도 {−10, 0, +10}° = 45개 조건.

| 측정 잡음 | 수렴 | 시간 평균 / 최대 | 최종 위치 오차 최대 | 최종 각도 오차 최대 |
|---|---|---|---|---|
| 없음 | 45/45 | 9.6 s / 18.1 s | 5.3 mm | 2.9° |
| 2 mm · 0.2° | 45/45 | 9.3 s / 18.1 s | 10.7 mm | 2.8° |
| 4 mm · 0.4° | 45/45 | 9.1 s / 17.9 s | 10.9 mm | 2.7° |
| 4 mm · 0.4°, **advance 0.10 m**, 오도메트리 1% 오차, 0.17 m 이내 마커 안 보임 | 45/45 | – / 22.4 s | 앞뒤 12.1 mm · 좌우 15.0 mm | 2.7° |

> 개발 중 확인한 문제: 처음 쓴 극좌표 제어(Siegwart)는 **좌우로만 어긋난 경우** 앞뒤로 진동하며
> 45개 중 20개 실패 → 3단계 방식으로 변경. 또 완료 판정을 접근 중에도 하면 허용 오차 경계(≈13 mm)에서
> 멈추는 문제가 있어, 접근을 끝낸 뒤(ROTATE)에만 완료 판정하도록 수정.

### 주의
- **Nav2 목표가 끝난 뒤에만** 호출 (Nav2와 `/cmd_vel` 충돌)
- 정렬 중 팔은 **접은 자세** (Pi 카메라 시야 가림 · 통과 충돌 방지)
- Pi 카메라 수평 화각 약 ±31°. Nav2 도착 오차가 각도 10° + 좌우 5 cm면 마커가 약 26°까지 벗어남 →
  더 크면 Nav2 도착 지점을 통 정면에 더 가깝게 잡거나 `standoff`를 늘린다
- 실기에서는 바퀴 미끄러짐 · 지연이 있으므로 `max_v`를 낮춰 시작하고 게인을 조정

---

## 9. 진행 계획 (팀 가이드 순서에 맞춤)

| 단계 | 내용 | 관련 |
|---|---|---|
| 0 | 두 보드 연동: 라파 유선 고정 IP 192.168.10.1 · Cyclone DDS · chrony · Orin에서 `/aruco/*` 수신 | 팀 가이드 |
| 1 | 라파에서 Pi 카메라 발행 → 캘리브레이션 → 라파에서 `aruco_node` 실행 | 5.1, 5.3 |
| 2 | Pi 카메라 TF 확인 → `aruco_node` 실측 보정 | 5.4, 7 |
| 3 | 쓰레기통 정렬 실기 테스트: 일부러 5 cm · 10° 어긋나게 세우고 `auto_start` | 8 |
| 4 | 팔 · D435i TF 통합 (팀 가이드 9장) | 아래 |
| 5 | 놓기 동작 (`arm_server.py` 의 `/drop/place`) 실기 확인 | [`../turtlebot3_mission`](../turtlebot3_mission/README.md) |
| 6 | `mission_node.py` 단계별 리허설 → 전체 | [`../turtlebot3_mission`](../turtlebot3_mission/README.md) |
| 7 | `wall_localizer`: 주요 포인트 마커(ID 1·2)로 AMCL 위치 보정, 거치대 마커(ID 3)로 접근 보정 | TODO |

### 4단계 메모: D435i가 팔에 달려 있을 때
- TF: `base_link → (팔 마운트, static) → link1 → … → link5 (joint_states) → d435i_link (static, 실측)`
  → 팔이 움직이면 카메라 위치도 따라 바뀐다. 팔 robot_state_publisher가 정상이어야 YOLO 좌표가 맞다
- 바퀴와 팔이 모두 OpenCR → Orin(`turtlebot3_manipulation`)으로 구동되므로 `/joint_states` 는 하나로 나온다 (분리 불필요)
- 손목 카메라 위치·그리퍼 방향 등 실측값은 [`pick&place`](https://github.com/hanjaejin124/turtlebot/blob/main/pick%26place/README.md) 참고
- **관측 자세**: 팔을 정해진 자세로 들어 쓰레기를 내려다보고 YOLO + 뎁스로 위치를 구한 뒤, 그 값으로 집기
  (D435i는 약 0.2~0.3 m보다 가까우면 뎁스가 안 나오므로 접근 중에는 다시 보지 않는다)
- 카메라 장착 위치는 자로 실측하거나, 마커를 이용한 hand-eye 캘리브레이션으로 구한다

### 놓기 · 시나리오 노드
[`../turtlebot3_mission/`](../turtlebot3_mission/README.md) 에 작성 (`mission_node.py`, `arm_server.py`, `omx_ik.py`). 요점:
- 놓는 높이는 **그리퍼 중심 바닥 기준 0.30 m** (통 입구 20 cm + 옆으로 잡은 캔 절반 6 cm + 여유) — `bin_center` 의 z는 쓰지 않음
- 통 위로 갈 때 5 cm 더 띄운 경유점, MoveIt 장면에 쓰레기통 원통 추가
- 통 위치는 `odom` 기준으로 기억 → 정렬 후 직진(advance)으로 마커가 안 보여도 놓을 수 있음

### TODO 노드
- **wall_localizer**: 주요 포인트 마커(ID 1·2) 지도 좌표 `WALL_MAP = {1: (x, y, yaw), ...}` 기록 →
  `T_map_base = T_map_marker × inv(T_base_marker)` → `/initialpose` 발행 (정지 상태, 1.5 m 이내, 쿨다운)

---

## 10. 트러블슈팅

| 증상 | 원인 · 해결 |
|---|---|
| `/aruco/*` 가 발행되지 않음 | `camera_info` 미수신 → `camera_info_url` 확인 |
| Orin에서 화면용 `/camera/image_raw/compressed` 가 없음 | 라파에 `ros-humble-image-transport-plugins` 설치 |
| Orin에서 `/aruco/*` 가 안 보임 | 라파 유선 IP(192.168.10.1) 설정, DOMAIN_ID, RMW, cyclonedds.xml 인터페이스 (팀 가이드 10장) |
| 거리가 일정 비율로 틀림 | `size` 실측값 오류 또는 인쇄 배율 |
| 인식이 안 됨 | 반사, 흰 여백 부족, 마커가 너무 작음(30 px 미만), 카메라와 높이 차이 |
| `bin_aligner` 에서 TF 변환 실패 | base_link → 카메라 프레임 TF 없음 (5.4) |
| 정렬 중 "마커를 놓침" | 마커가 화각 밖 → Nav2 도착 지점 조정, `psi_max_deg` 축소 |
| 정렬이 떨리거나 넘침 | `max_v`, `k_psi`, `k_lat` 낮추기 |
| TF extrapolation 에러 | chrony 동기화 (`chronyc tracking`) |
