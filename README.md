# TurtleBot3 쓰레기 수거 로봇 — ArUco · 시연 시나리오

TurtleBot3 Waffle Pi + OpenMANIPULATOR-X에 Jetson Orin Nano를 더해, 복도에서 쓰레기(콜라캔)를 주워
ArUco 마커가 붙은 쓰레기통에 버리는 시연 프로젝트입니다. 이 저장소는 그중 **ArUco 마커 인식 · 정렬 · 시연 시나리오** 부분을 담고 있습니다.

- 학부 수업 · 3인 팀 · 2026.09 ~
- Ubuntu 22.04 · ROS 2 Humble · slam_toolbox · Nav2 · MoveIt 2 · YOLO26n (TensorRT) · OpenCV ArUco
- 팀 통합 저장소: [hanjaejin124/turtlebot](https://github.com/hanjaejin124/turtlebot) — SLAM · Nav2(`slam_nav2/`), 집기(`pick&place/`)는 그쪽에 있습니다.

## 시연 시나리오

시연 장소는 SLAM으로 미리 지도를 만들어 두고, 시연 때는 그 지도로 자율주행합니다.

```
① 출발         ② 이동 · 위치 보정            ③ 줍기                          ④ 이동            ⑤ 버리기
출발점에서  →  Nav2 자율주행               →  캔 거치대(ArUco ID 3)로 접근  →  Nav2 자율주행  →  쓰레기통(ArUco ID 0) 정렬
AMCL 초기화    주요 포인트 ArUco(ID 1·2)로      D435i + YOLO로 캔 인식           쓰레기통 지점      → 캔 투입
               위치 정확도 보정                OpenMANIPULATOR-X로 집기
```

## 하드웨어 · 보드 역할

| 보드 | 역할 | 연결 |
|---|---|---|
| **Jetson Orin Nano** (메인, JetPack 6.2.1) | SLAM · Nav2 · YOLO(D435i) · MoveIt 2 · 정렬 노드 · 시나리오 진행 | OpenCR(USB) · LDS-03(USB) · D435i(USB) · 라파(유선 랜) |
| **OpenCR** | Orin의 명령을 다이나믹셀 모터(팔 · 그리퍼 · 바퀴)로 전달 | Orin과 USB |
| **Raspberry Pi 4** | **Pi 카메라 + ArUco 마커 인식만** 담당 → 결과(`/aruco/*`)를 Orin으로 | Orin과 유선 랜 |

| 항목 | 내용 |
|---|---|
| 카메라 | Intel RealSense D435i (OMX 손목 장착, YOLO · 뎁스), Raspberry Pi Camera v2.1 (몸체 고정, ArUco) |
| 통신 | 라파 ↔ Orin 유선 직결 (192.168.10.1 ↔ 192.168.10.2, Cyclone DDS, ROS_DOMAIN_ID=30), 인터넷 · SSH는 각 보드 Wi-Fi |
| 시간 동기화 | 시연장은 오프라인 가정 → Orin(chrony)이 시간 서버, 라파가 맞춤 (`pi_tools/time_sync/`) |

집는 순간 손목의 D435i 시야는 그리퍼와 캔에 가려지기 때문에, 마커 인식은 몸체에 고정된 Pi 카메라로 따로 합니다.

## ArUco 마커

`DICT_4X4_50`, 인쇄물 · 생성 코드는 [`turtlebot3_aruco`](turtlebot3_aruco/README.md).

| ID | 위치 | 크기 | 용도 |
|---|---|---|---|
| 0 | 원형 쓰레기통 (지름 19 cm, 높이 20 cm) | 60 mm (원통 곡면 보정) | 쓰레기통 앞 정렬 → 투입 |
| 1, 2 | 주요 포인트 (평면) | 80 mm | 이동 중 위치 정확도 보정 |
| 3 | 콜라캔 거치대 (평면) | 80 mm | 거치대 접근 정확도 보정 |

## 구조

```
turtlebot3_aruco/    마커 인식(라파), 쓰레기통 · 캔 거치대 정렬, 마커 위치 보정, SLAM 자율 탐색, 인쇄용 마커
turtlebot3_mission/  시연 시나리오 상위 노드, 팔 서비스(관측 · 접기 · 집기 · 놓기), OMX IK
pi_tools/            라파 카메라 · ArUco 실행 스크립트, Pi 카메라 캘리브레이션, 시간 동기화 설정
```

| 폴더 | 내용 | 상태 |
|---|---|---|
| [`turtlebot3_aruco`](turtlebot3_aruco/README.md) | 마커 인식 노드(원통 곡면 보정, 라파에서 실행), 쓰레기통 · 캔 거치대 정렬 노드(+직진, Orin), 마커 위치 보정(`marker_mapper` · `aruco_localizer`), 자율 탐색(`aruco_explore`), 인쇄물 | 라파 인식 실기 확인, 정렬은 시뮬레이션 검증 |
| [`turtlebot3_mission`](turtlebot3_mission/README.md) | `mission_node` (12단계 · 재시도 · 안전 정지), `arm_server` (MoveIt · 그리퍼 · grasp_node 연동), OMX IK | 코드 작성 · 로직 검증, 실기 테스트 전 |
| `pi_tools` | `start_camera.sh` → `start_aruco.sh`, `pi_camera.yaml`, chrony · timesyncd 설정 | 라파에서 사용 중 |

## 진행 상황

- [x] Jetson Orin Nano 환경 구축 (NVMe 부팅, ROS 2 Humble, MoveIt 2)
- [x] YOLO26n TensorRT FP16 변환 (추론 37.8 ms → 10.9 ms)
- [x] ArUco 마커 인식 노드 · 인쇄물
- [x] 쓰레기통 정렬 노드 (시뮬레이션 45/45 수렴)
- [x] 전체 시나리오 상위 노드 · 팔 서비스 코드 (로직 검증)
- [x] 라파 ↔ Orin 유선 연동 (Cyclone DDS) · 오프라인 시간 동기화
- [x] Pi 카메라 캘리브레이션 · 라파에서 마커 인식 확인 (`pi_tools/start_camera.sh` → `pi_tools/start_aruco.sh`)
- [x] 캔 거치대(ID 3) 정렬 노드 `stand_aligner` + `mission_node` 'approach' 단계 (시뮬레이션 ±5 cm 39/40, ±10 cm 36/40)
- [x] 마커로 위치 보정: 맵핑 중 마커 지도 좌표 기록 · 웨이포인트 자동 생성(`marker_mapper`), 주행 중 AMCL 보정(`aruco_localizer`)
- [x] SLAM 자율 탐색 `aruco_explore` (frontier를 Nav2로 탐색 → 지도가 다 그려지면 자동 저장 · 종료, 마커 = 맵의 끝)
- [ ] 시연 장소 맵핑 · 위치 보정 실기 확인
- [ ] 캔 거치대 정렬 실기 테스트 (`can_offset` 실측)
- [ ] 쓰레기통 정렬 실기 테스트 · 놓기
- [ ] 집기는 팀 저장소 [`pick&place`](https://github.com/hanjaejin124/turtlebot/blob/main/pick%26place/README.md) 결과(그리퍼 방향 · 도착 확인 · 카메라 보정)에 맞춰 `arm_server` 연동
- [ ] 복도 리허설
