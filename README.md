# TurtleBot3 쓰레기 수거 로봇

TurtleBot3 Waffle Pi + OpenMANIPULATOR-X로 복도에서 쓰레기(콜라캔)를 주워 지정한 쓰레기통에 버리는 시연 프로젝트입니다.

- 학부 수업 · 3인 팀 · 2026.09 ~
- Ubuntu 22.04 · ROS 2 Humble · Nav2 · MoveIt 2 · YOLO26n (TensorRT) · OpenCV ArUco

## 시연 시나리오

```
① 출발          ② 이동            ③ 줍기                 ④ 이동            ⑤ 버리기
교수님 방 앞  →  Nav2 자율주행  →  D435i + YOLO 인식    →  Nav2 자율주행  →  Pi 카메라 ArUco 인식
초기 위치 지정    → 쓰레기 지점      → OMX로 집기             → 쓰레기통 지점    → 정렬 후 투입
```

## 하드웨어 구성

| 항목 | 내용 |
|---|---|
| 플랫폼 | TurtleBot3 Waffle Pi |
| 매니퓰레이터 | OpenMANIPULATOR-X (4축) |
| 메인 컴퓨터 | Jetson Orin Nano (JetPack 6.2.1) — bringup · Nav2 · MoveIt · YOLO |
| 보조 컴퓨터 | Raspberry Pi 4 — Pi 카메라 · ArUco 인식 |
| 카메라 | Intel RealSense D435i (YOLO · 뎁스), Raspberry Pi Camera v2.1 (ArUco) |

집는 순간 D435i 시야가 그리퍼와 물체에 가려지기 때문에, 마커 인식은 Pi 카메라로 따로 합니다.

## 구조

```
turtlebot3_aruco/    Pi 카메라 ArUco 마커 인식 (쓰레기통 1 · 벽 3), 인쇄용 마커, 설정 가이드
```

| 폴더 | 내용 | 상태 |
|---|---|---|
| [`turtlebot3_aruco`](turtlebot3_aruco/README.md) | 마커 인식 노드, 원통 곡면 보정, 인쇄물, 네트워크 · 캘리브레이션 가이드 | 코드 작성 · 실기 테스트 전 |

## 진행 상황

- [x] Jetson Orin Nano 환경 구축 (NVMe 부팅, ROS 2 Humble, MoveIt 2)
- [x] YOLO26n TensorRT FP16 변환 (추론 37.8 ms → 10.9 ms)
- [x] MoveIt 집기 시뮬레이션 (4개 시나리오 검증)
- [x] ArUco 마커 인식 노드 · 인쇄물
- [ ] Pi 카메라 캘리브레이션 · 실기 마커 인식 확인
- [ ] 쓰레기통 정렬 · 놓기 노드
- [ ] 전체 시나리오 상위 노드 (Nav2 → 집기 → 이동 → 버리기)
- [ ] 복도 리허설
