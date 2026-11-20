#!/bin/bash
# R3LIVE 真数据: hku_park_01 (Livox avia, 351s)
set -x
source /opt/ros/noetic/setup.bash
source /root/ws7/devel/setup.bash

export HOME=/root
mkdir -p /root/r3live_output

roscore > /tmp/roscore.log 2>&1 &
sleep 4

roslaunch r3live r3live_bag.launch rviz:=0 > /data/r3live_park01_ros.log 2>&1 &
RL=$!
sleep 12

rosbag record -O /data/r3live_park01_odom.bag /aft_mapped_to_init > /tmp/record.log 2>&1 &
REC=$!
sleep 2
rosbag play /data/modality_D/hku_park_01.bag > /data/r3live_park01_play.log 2>&1 &
PLAY=$!
wait $PLAY
sleep 10

kill -INT $REC 2>/dev/null; sleep 3
kill $RL 2>/dev/null; sleep 3
pkill -f r3live 2>/dev/null

echo "===== r3live.log 尾部 ====="
tail -20 /tmp/r3live.log
echo "===== 输出 ====="
find /root/r3live_output -maxdepth 2 2>/dev/null | head -10
echo "R3LIVE PARK01 DONE"
