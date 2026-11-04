#!/bin/bash
# KITTI + LIO-SAM 运行 + 轨迹录制（headless）
BAG="${BAG:?env BAG required}"; TAG="${TAG:?env TAG required}"
set -x
source /opt/ros/noetic/setup.bash
source /root/ws4/devel/setup.bash
roscore > /tmp/roscore.log 2>&1 &
sleep 4
roslaunch lio_sam run_kitti.launch > /tmp/liosam.log 2>&1 &
LAUNCH=$!
sleep 12
rosbag record -O /data/kitti/lsam_${TAG}.bag /aft_mapped_to_init > /tmp/record.log 2>&1 &
REC=$!
sleep 1
rosbag play --clock "$BAG" > /tmp/play.log 2>&1
sleep 3
kill -INT $REC 2>/dev/null; sleep 5
kill $LAUNCH 2>/dev/null
tail -4 /tmp/liosam.log
