#!/bin/bash
# 라파: ArUco 인식 노드 시작 (카메라가 켜져 있어야 함: start_camera.sh). 결과 /aruco/* -> Orin
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
pkill -f "turtlebot3_aruco/aruco_node.py" 2>/dev/null; sleep 1
cd ~/turtlebot/aruco/turtlebot3_aruco
nohup python3 aruco_node.py > /tmp/pi_aruco.log 2>&1 &
echo "aruco_node started (log /tmp/pi_aruco.log)"
