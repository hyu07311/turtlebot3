#!/bin/bash
# 라파: Pi 카메라 시작 (센서 전체 화각 1640x1232 → 640x480 출력). 사용: start_camera.sh [camera_info yaml]
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
pkill -f "camera_ros/camera_node" 2>/dev/null; sleep 1
ARGS="-p width:=640 -p height:=480 -p format:=RGB888 -p sensor_mode:=1640:1232 -p frame_id:=camera_rgb_optical_frame"   # frame_id: 로봇 URDF 의 Pi 카메라 광학 프레임
INFO="${1:-$HOME/pi_camera.yaml}"   # 보정 파일 (기본 ~/pi_camera.yaml)
[ -f "$INFO" ] && ARGS="$ARGS -p camera_info_url:=file://$INFO"
nohup ros2 run camera_ros camera_node --ros-args $ARGS > /tmp/pi_camera.log 2>&1 &
echo "camera started (log /tmp/pi_camera.log)"
