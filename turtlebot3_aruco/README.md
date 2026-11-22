# TurtleBot3 ArUco 마커 인식 · 쓰레기통 정렬

TurtleBot3 Waffle Pi + OpenMANIPULATOR-X(RM-X52-TNM) 쓰레기 수거 시연에서 사용하는 ArUco 마커 인식,
쓰레기통 앞 정렬 코드와 인쇄물.

- **시나리오**: 출발 → Nav2 이동 → 쓰레기 줍기(D435i + YOLO) → Nav2 이동 → **쓰레기통 마커로 정렬** → 버리기
- **카메라 역할**
  - D435i: **OMX 팔에 장착** (eye-in-hand) → YOLO 인식 · 뎁스. 집는 순간 시야가 가려짐
  - Pi 카메라: **로봇 몸체에 고정** → ArUco 마커 인식 (쓰레기통 · 벽)
- 보드 구성 · 네트워크 · 팔 구동은 팀 통합 세팅 가이드(`터틀봇3 + OpenMANIPULATOR-X + Orin Nano 통합 세팅 가이드`)를 따른다.
  이 문서는 그 위에서 ArUco 관련 부분만 다룬다.

---

## 1. 시스템 구성 (팀 가이드 기준)

```
[스마트폰 핫스팟] -- Wi-Fi -- 라파4, Orin Nano, (내 PC)   : 인터넷, SSH
라파4 (192.168.10.1) == 유선랜 직결 == Orin Nano (192.168.10.2) : ROS 2 통신 전용 (Cyclone DDS)

[라파4]                                         [Orin Nano]
 turtlebot3_bringup (OpenCR, LDS-03, odom)       팔 (U2D2, ros2_control, MoveIt)
 camera_ros: Pi 카메라 ─ /camera/image_raw/compressed ─▶ aruco_node.py ─▶ /aruco/*
                ◀──────────────── /cmd_vel ──────────── bin_aligner.py
                                                 D435i (팔 장착) + YOLO, Nav2, 작업 로직
```

| 항목 | 내용 |
|---|---|
| ROS | ROS 2 Humble, `ROS_DOMAIN_ID=30`, `rmw_cyclonedds_cpp` (유선 인터페이스 고정) |
| Pi 카메라 | Raspberry Pi Camera v2.1 (IMX219), 640×480, 라파4에서 발행 |
| ArUco 처리 | **Orin** (라파는 주행 · 라이다 담당이라 부하를 Orin으로) |
| 마커 사전 | `DICT_4X4_50` |

---

## 2. 파일 구조

```
.
├── aruco_node.py        # 마커 인식 (Orin, Pi 카메라 압축 영상 구독)
├── bin_aligner.py       # 쓰레기통 앞 미세 정렬 (Orin, /cmd_vel)
├── make_markers.py      # 인쇄용 마커 PDF 생성
└── markers/
    ├── aruco_markers_60mm.pdf   # ID 0 (쓰레기통)       ← 인쇄
    ├── aruco_markers_80mm.pdf   # ID 1, 2, 3 (벽)       ← 인쇄
    └── *.png                    # 미리보기
```

---

## 3. 마커 구성

| ID | 부착 위치 | 크기 | 비고 | 발행 토픽 |
|---|---|---|---|---|
| 0 | 원형 쓰레기통 (지름 19 cm, 높이 20 cm) | 60 mm | 원통 곡면 보정 | `/aruco/bin_pose`, `/aruco/bin_center` |
| 1 | 벽 | 80 mm | 평면 | `/aruco/wall_1_pose` |
| 2 | 벽 | 80 mm | 평면 | `/aruco/wall_2_pose` |
| 3 | 벽 | 80 mm | 평면 | `/aruco/wall_3_pose` |

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
  -p camera_info_url:=file:///home/ubuntu/pi_camera.yaml
```
- `image-transport-plugins` 가 있어야 `/camera/image_raw/compressed` 가 발행된다.
- 카메라 미인식 시 `/boot/firmware/config.txt` 에 `dtoverlay=imx219` 추가 후 재부팅.
- 최신 터틀봇3는 Pi 카메라를 도커로 지원하는 변경이 있으니 e-Manual도 확인 (팀 가이드 5.7).

### 5.2 Orin: 패키지
```bash
sudo apt install -y ros-humble-cv-bridge ros-humble-tf2-geometry-msgs python3-scipy
```

### 5.3 Pi 카메라 캘리브레이션 (필수)
캘리브레이션 값(`camera_info`)이 없으면 `aruco_node`는 아무것도 발행하지 않는다.
```bash
# Orin(모니터 연결)에서. 체커보드 8x6 내부코너, 한 칸 25 mm 예시
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -p camera:=/camera
```
보드를 여러 위치·각도·거리로 움직여 X/Y/Size/Skew 바가 초록색이 되면 **CALIBRATE → SAVE** →
yaml을 라파로 복사(`~/pi_camera.yaml`)해서 `camera_info_url` 로 지정.

### 5.4 Pi 카메라 TF
몸체 고정이므로 `base_link → 카메라 광학 프레임` 은 고정값이다.
- 터틀봇3 와플파이 URDF(라파의 robot_state_publisher)에 이미 `camera_rgb_optical_frame` 이 있다.
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
# 라파: 베이스 + 카메라
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 \
  -p camera_info_url:=file:///home/ubuntu/pi_camera.yaml

# Orin: 마커 인식
python3 aruco_node.py
ros2 topic hz /aruco/bin_pose          # 10 Hz 이상
ros2 topic echo /aruco/wall_1_pose     # 30 cm 앞에 두면 position.z ≈ 0.30

# Orin: 쓰레기통 정렬 (Nav2 도착 후)
python3 bin_aligner.py --ros-args -p advance:=0.12   # 직진 거리는 팔 마운트에 맞게
ros2 service call /bin_aligner/align std_srvs/srv/Trigger
```
라파에서 직접 인식하려면 `aruco_node.py` 의 `USE_COMPRESSED = False`.

---

## 7. `aruco_node.py` — 마커 인식

### 설정 (이 부분만 수정)
```python
MARKERS = {
    0: dict(name="bin",    size=0.060, cyl_radius=0.095),  # 원형 쓰레기통 (지름 19 cm)
    1: dict(name="wall_1", size=0.080),
    2: dict(name="wall_2", size=0.080),
    3: dict(name="wall_3", size=0.080),
}
OFFSET = {}            # 실측 보정값 (m, 카메라 광학 프레임) 예: {0: (0.0, 0.0, 0.004)}
AVG_N = 5              # 이동 평균 프레임 수
USE_COMPRESSED = True  # Orin: 압축 영상 구독 / 라파에서 실행: False
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
| 0 | 두 보드 연동 (팀 가이드 4~8장): 유선 · Cyclone DDS · chrony · Orin에서 teleop | 팀 가이드 |
| 1 | Pi 카메라 발행(압축) → Orin 수신 확인 → 캘리브레이션 | 5.1, 5.3 |
| 2 | Pi 카메라 TF 확인 → `aruco_node` 실측 보정 | 5.4, 7 |
| 3 | 쓰레기통 정렬 실기 테스트: 일부러 5 cm · 10° 어긋나게 세우고 `auto_start` | 8 |
| 4 | 팔 · D435i TF 통합 (팀 가이드 9장) | 아래 |
| 5 | 놓기 동작 (`arm_server.py` 의 `/drop/place`) 실기 확인 | [`../turtlebot3_mission`](../turtlebot3_mission/README.md) |
| 6 | `mission_node.py` 단계별 리허설 → 전체 | [`../turtlebot3_mission`](../turtlebot3_mission/README.md) |
| 7 | (필요 시) `wall_localizer`: 벽 마커로 AMCL 위치 보정 | TODO |

### 4단계 메모: D435i가 팔에 달려 있을 때
- TF: `base_link → (팔 마운트, static) → link1 → … → link5 (joint_states) → d435i_link (static, 실측)`
  → 팔이 움직이면 카메라 위치도 따라 바뀐다. 팔 robot_state_publisher가 정상이어야 YOLO 좌표가 맞다
- 팀 가이드 9장대로 바퀴(라파)와 팔(Orin)의 `/joint_states` 를 분리하지 않으면 TF가 깨진다
- **관측 자세**: 팔을 정해진 자세로 들어 쓰레기를 내려다보고 YOLO + 뎁스로 위치를 구한 뒤, 그 값으로 집기
  (D435i는 약 0.2~0.3 m보다 가까우면 뎁스가 안 나오므로 접근 중에는 다시 보지 않는다)
- 카메라 장착 위치는 자로 실측하거나, 마커를 이용한 hand-eye 캘리브레이션으로 구한다

### 놓기 · 시나리오 노드
[`../turtlebot3_mission/`](../turtlebot3_mission/README.md) 에 작성 (`mission_node.py`, `arm_server.py`, `omx_ik.py`). 요점:
- 놓는 높이는 **그리퍼 중심 바닥 기준 0.30 m** (통 입구 20 cm + 옆으로 잡은 캔 절반 6 cm + 여유) — `bin_center` 의 z는 쓰지 않음
- 통 위로 갈 때 5 cm 더 띄운 경유점, MoveIt 장면에 쓰레기통 원통 추가
- 통 위치는 `odom` 기준으로 기억 → 정렬 후 직진(advance)으로 마커가 안 보여도 놓을 수 있음

### TODO 노드
- **wall_localizer**: 벽 마커 지도 좌표 `WALL_MAP = {1: (x, y, yaw), ...}` 기록 →
  `T_map_base = T_map_marker × inv(T_base_marker)` → `/initialpose` 발행 (정지 상태, 1.5 m 이내, 쿨다운)

---

## 10. 트러블슈팅

| 증상 | 원인 · 해결 |
|---|---|
| `/aruco/*` 가 발행되지 않음 | `camera_info` 미수신 → `camera_info_url` 확인 |
| Orin에서 `/camera/image_raw/compressed` 가 없음 | 라파에 `ros-humble-image-transport-plugins` 설치 |
| Orin에서 토픽이 안 보임 | 팀 가이드 10장 (DOMAIN_ID, RMW, cyclonedds.xml 인터페이스) |
| 거리가 일정 비율로 틀림 | `size` 실측값 오류 또는 인쇄 배율 |
| 인식이 안 됨 | 반사, 흰 여백 부족, 마커가 너무 작음(30 px 미만), 카메라와 높이 차이 |
| `bin_aligner` 에서 TF 변환 실패 | base_link → 카메라 프레임 TF 없음 (5.4) |
| 정렬 중 "마커를 놓침" | 마커가 화각 밖 → Nav2 도착 지점 조정, `psi_max_deg` 축소 |
| 정렬이 떨리거나 넘침 | `max_v`, `k_psi`, `k_lat` 낮추기 |
| TF extrapolation 에러 | chrony 동기화 (`chronyc tracking`) |
