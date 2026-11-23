# 시연 시나리오 상위 노드 (mission) · 팔 서비스

출발 → 쓰레기 줍기 → 쓰레기통에 버리기 전체를 순서대로 실행하는 노드와, 그에 필요한 팔 서비스 서버.
모두 **Orin**에서 실행한다. (라파는 Pi 카메라 + ArUco 인식만. 바퀴·팔은 OpenCR → Orin)

## 1. 구성

```
mission_node.py ─┬─ Nav2 navigate_to_pose ─────────────── 이동 (START / TRASH / BIN)
                 ├─ /arm/look, /arm/stow ──┐
                 ├─ /grasp/pick ───────────┼── arm_server.py ─┬─ MoveIt /move_action
                 ├─ /drop/place ───────────┘                  ├─ /gripper_controller/gripper_cmd
                 │                                            └─ grasp_node (기존, 입력만 remap)
                 └─ /bin_aligner/align ──────────────── bin_aligner.py ([../turtlebot3_aruco](../turtlebot3_aruco/README.md))
```

| 파일 | 역할 |
|---|---|
| `mission_node.py` | 시나리오 순서 실행, 단계별 재시도, 실패 시 안전 정지 |
| `arm_server.py` | 팔 서비스 4개 (관측 자세 · 접기 · 집기 · 놓기) |
| `omx_ik.py` | OpenMANIPULATOR-X 해석적 IK (놓기 동작용, ROS 의존성 없음) |

## 2. 시나리오

| 단계 | 내용 | 실패 시 |
|---|---|---|
| `init` | START 위치로 AMCL 초기화, Nav2 준비 확인 | 중단 |
| `go_trash` | Nav2 → TRASH (`waypoints.yaml` 에 `via: [P1]` 을 넣으면 P1 에 들러 마커로 위치 보정 후 이동) | 1회 재시도, AMCL로 도착 재확인 (0.4 m) |
| `approach` | `/stand_aligner/align` (거치대 마커 ID 3 정렬 + 직진 → 캔이 팔 앞 약 0.30 m) | TRASH 재이동 후 재시도 |
| `look` | 팔 관측 자세 (D435i가 앞쪽 바닥을 봄) | 중단 |
| `detect` | `/cola/target_point` 가 약 1초 연속으로 들어올 때까지 대기 | 제자리 +20° → −20° 돌아보며 재시도 |
| `pick` | `/grasp/pick` | 관측 자세 → 재인식 → 1회 재시도 |
| `stow` | 팔 접기 | 중단 |
| `go_bin` | Nav2 → BIN | 1회 재시도 |
| `align` | `/bin_aligner/align` (마커 정렬 + 직진) | BIN 재이동 후 1회 재시도 |
| `drop` | `/drop/place` | 중단 |
| `stow`, `go_start` | 팔 접기, 출발점 복귀 | 중단 |

어느 단계든 실패하거나 Ctrl+C를 누르면 **Nav2 이동 취소 → 바퀴 정지 → 팔 접기** 후 종료한다.

## 3. 준비

### 3.1 웨이포인트 ([`slam_nav2`](https://github.com/hanjaejin124/turtlebot/blob/main/slam_nav2/README.md) 의 `waypoint_nav.py`)
시연 장소는 미리 SLAM으로 맵핑해 `~/slam_nav2/maps/map.yaml` 로 저장해 둔다.
```bash
python3 ~/slam_nav2/scripts/waypoint_nav.py record     # RViz Publish Point로 3곳 클릭
nano ~/slam_nav2/waypoints.yaml                        # 이름을 START / TRASH / BIN 으로, yaw(도) 수정
```
```yaml
- {name: START, x: 0.0, y: 0.0, yaw: 0.0}
- {name: TRASH, x: 3.2, y: 0.4, yaw: 0.0}     # 캔 거치대(ArUco ID 3) 앞, 캔이 D435i 관측 범위에 들어오는 위치·방향
- {name: BIN,   x: 5.1, y: -0.6, yaw: 90.0}   # 쓰레기통 정면 약 0.3 m 앞, 통을 바라보는 방향
```

### 3.2 grasp_node 연동 (팀원 코드 — 실행 인자와 2줄만 추가)
1. 입력 토픽을 바꿔서 YOLO가 직접 집기를 시작하지 않게 한다 (arm_server가 안정된 목표 1개만 넘김)
   ```bash
   ros2 run omx_cola_grasp grasp_node --ros-args -r /cola/target_point:=/grasp/target -p execute:=true
   ```
2. 집기 순서(열기 → 접근 → 집기 → 닫기 → 들기)가 끝나는 곳에서 결과 발행
   ```python
   from std_msgs.msg import Bool
   self.result_pub = self.create_publisher(Bool, '/grasp/result', 10)   # __init__
   self.result_pub.publish(Bool(data=success))                          # 집기 끝
   ```
   2를 아직 못 했으면 `arm_server` 를 `-p grasp_result_topic:=""` 로 실행 → '팔이 움직였다 멈춤'으로 완료를 추정.

### 3.3 arm_server 파라미터 (⚠ 실기에서 확인)
| 이름 | 기본값 | 확인 방법 |
|---|---|---|
| `look_pose` | `[0.0, -0.3, 0.5, 1.1]` | RViz MotionPlanning에서 D435i가 TRASH 앞 바닥을 보게 맞춘 관절값 |
| `stow_pose` | `[0.0, -1.0, 0.3, 0.7]` | 팔을 접고 Pi 카메라 시야를 가리지 않는 자세 |
| `gripper_open` | `0.019` | 그리퍼 열림 위치 (turtlebot3_manipulation 설정 확인) |
| `gripper_empty_below` | `0.0` | 빈손으로 닫았을 때 그리퍼 관절값보다 약간 큰 값 (실측) |
| `drop_height` | `0.30` m | 놓을 때 그리퍼 중심 높이 (바닥 기준) — 아래 4장 |
| `bin_height`, `bin_radius` | `0.20`, `0.095` m | MoveIt 장면에 넣는 쓰레기통 크기 |
| `group`, `/move_action`, `gripper_action` | `arm`, …, `/gripper_controller/gripper_cmd` | 설치된 MoveIt 설정 이름 |
| `joint_states_topic` | `/joint_states` | 팀 가이드 9장처럼 팔 쪽 이름을 분리했다면 수정 |

## 4. 놓기 동작과 도달 범위 (중요)

### 놓는 높이 0.30 m
캔(높이 약 12 cm)을 옆에서 가운데를 잡으면 캔 아래 끝이 그리퍼 중심보다 약 6 cm 아래로 내려온다.
통 입구 20 cm + 캔 절반 6 cm + 여유 4 cm → **그리퍼 중심 0.30 m**. 통 위로 갈 때는 5 cm 더 띄운 경유점을 지난다.
MoveIt 계획 장면에 쓰레기통 원통을 넣어 팔이 테두리에 부딪히지 않게 계획한다.

### 팔이 닿는 거리 → 정렬 후 직진(advance)
OMX는 이 높이에서 joint1 축으로부터 약 0.35 m까지 닿는다.
turtlebot3_manipulation 표준 마운트(`link1` = `base_link` + (−0.092, 0, 0.091))를 가정하면:

| 항목 | 값 |
|---|---|
| 통 표면이 있어야 하는 거리 | `base_link` 에서 **0.153 m 이내** |
| 와플 앞면과 통 사이 | 약 5~7 cm |
| `standoff` 0.25 m로 정렬 후 필요한 직진 | **advance ≥ 0.10 m (여유 포함 0.12 m)** |

0.15 m까지 다가가면 Pi 카메라(고정 초점)에는 마커가 너무 가깝고 흐리다. 그래서
**잘 보이는 0.25 m에서 정렬 → 오도메트리로 0.12 m 직진**하고, `arm_server` 는 마지막으로 본 통 위치를
`odom` 기준으로 기억해 둔 값으로 놓는다.

> 팔 마운트 위치가 다르면 값이 달라진다. 범위 밖이면 `/drop/place` 가
> "link1에서 수평 X m, 최대 약 Y m → standoff 를 Z m 이상 줄이세요" 라고 알려 준다.

## 5. 실행

```bash
# 라파 — Pi 카메라 + ArUco 인식만
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p camera_info_url:=file:///home/turtlebot/pi_camera.yaml
python3 turtlebot3_aruco/aruco_node.py                            # 라파용 기본값

# Orin — 각각 터미널
ros2 launch turtlebot3_manipulation_bringup base.launch.py        # OpenCR: 바퀴·팔 (pick&place 참고)
~/slam_nav2/lidar.sh                                              # LDS-03 (Orin USB)
ros2 launch turtlebot3_manipulation_moveit_config move_group.launch.py use_sim:=false
~/slam_nav2/nav2.sh                                               # 미리 만든 지도 ~/slam_nav2/maps/map.yaml
python3 turtlebot3_aruco/bin_aligner.py --ros-args -p advance:=0.12
python3 turtlebot3_aruco/stand_aligner.py                        # 캔 거치대(ID 3) 정렬
ros2 run omx_cola_grasp cola_detector --ros-args -p model_path:=/home/orin/yolo/yolo26n.engine -p "target_class:=cup,bottle" -p conf_threshold:=0.25
# (위 bringup · MoveIt · cola_detector · grasp_node 를 한 번에: ros2 launch omx_cola_grasp system.launch.py mission:=true rviz:=false)
ros2 run omx_cola_grasp grasp_node --ros-args -r /cola/target_point:=/grasp/target -p execute:=true
python3 turtlebot3_mission/arm_server.py

# 시나리오
python3 turtlebot3_mission/mission_node.py --list                 # 단계 목록
python3 turtlebot3_mission/mission_node.py                        # 전체
python3 turtlebot3_mission/mission_node.py go_bin align drop stow # 일부만 (리허설)
```

## 6. 리허설 순서 (단계를 끊어서)

| 순서 | 명령 | 확인 |
|---|---|---|
| 1 | `arm_server` 만 띄우고 `ros2 service call /arm/look std_srvs/srv/Trigger` | 관측 · 접기 자세, D435i 시야 |
| 2 | `mission_node.py look detect pick stow` (캔을 팔 앞에 직접 둠) | 집기 · 빈손 판정 |
| 2-1 | `mission_node.py go_trash approach look detect pick stow` | 거치대 정렬 후 캔이 집기 범위(0.29~0.33 m) 안 |
| 3 | `mission_node.py go_bin align` | Nav2 도착 → 정렬 → 직진 |
| 4 | 캔을 손으로 쥐여 주고 `mission_node.py drop stow` | 놓는 높이 · 통 충돌 |
| 5 | `mission_node.py` | 전체 |

## 7. 검증

ROS 실기 실행 전, 로직만 검증한 결과.

| 대상 | 방법 | 결과 |
|---|---|---|
| `omx_ik.py` | FK(0) = URDF 치수 합 / 12주차 시뮬레이션 목표 4개 / 무작위 FK→IK 왕복 | (0.286, 0, 0.2045) 일치 / 3개 도달 · 0.45 m 거부 (12주차와 동일) / 12,988개 오차 1e-16 m |
| `bin_aligner.py` (advance 0.12 포함) | 실제 `control_step()` 을 차동 구동 모델에 연결, 초기 오차 45조건, 측정 잡음 4 mm, 오도메트리 1% 오차, 0.17 m 이내 마커 안 보임 | 45/45 성공, 최대 23 s, 최종 오차 앞뒤 12 mm · 좌우 15 mm · 2.7° |
| `stand_aligner.py` | 같은 차동 구동 시뮬레이션, Nav2 도착 오차 무작위 40회 | ±5 cm → 39/40, ±10 cm → 36/40 이 집기 범위 안 |
| `mission_node.py` | ROS를 모의 객체로 바꿔 단계 순서 확인 | 정상 11단계 / 집기 실패 → 재인식 후 재시도 / 정렬 2회 실패 → 정지 · 팔 접기 / 일부 단계 실행 / 잘못된 단계 거부 |

`arm_server.py` 는 MoveIt · 그리퍼 · TF 와 직접 붙는 코드라 실기에서 처음 확인해야 한다 (문법 검사만 통과).
