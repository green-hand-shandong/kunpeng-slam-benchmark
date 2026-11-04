#!/bin/bash
# Cartographer 冒烟测试 v2：加静态 TF（velodyne→imu_link，来自 calib_imu_to_velo 取逆）
set -x
source /opt/ros/noetic/setup.bash
source /ws2/devel_carto/setup.bash

BAG=/data/kitti/kitti_0001.bag
OUT=/data/carto_smoke
mkdir -p $OUT

roscore &
sleep 3

# 静态 TF：imu_link ← velodyne（calib_imu_to_velo 的逆：R 转置、T 取 -R^T*t）
# calib_imu_to_velo: R≈I(小角度), t=(-0.809, 0.320, -0.800) 是 imu→velo
# velo→imu: t' = -R^T*t ≈ (0.809, -0.307, 0.803)（与 FAST-LIO 的 extrinsic_T 一致）
rosrun tf2_ros static_transform_publisher \
  0.8086759 -0.3195559 0.7997231 \
  -0.0010176 0.0010122 0.0003777 0.9999988 \
  imu_link velodyne > $OUT/tf_velodyne.log 2>&1 &
sleep 1
# IMU 数据本身的 frame 就是 imu，把它挂到 imu_link（同位置，恒等）
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 imu_link imu > $OUT/tf_imu.log 2>&1 &
sleep 1

/ws2/devel_carto/lib/cartographer_ros/cartographer_node \
  -configuration_directory /ws2/cfg \
  -configuration_basename kitti.lua \
  points2:=/velodyne_points \
  imu:=/kitti/oxts/imu \
  __name:=carto_smoke > $OUT/carto_node.log 2>&1 &
NODE_PID=$!
sleep 8

rosbag record -O $OUT/tracked_pose.bag /tracked_pose > $OUT/record.log 2>&1 &
REC_PID=$!
sleep 1

timeout 35 rosbag play --clock -u 30 $BAG > $OUT/play.log 2>&1
PLAY_RC=$?

sleep 5
kill -INT $REC_PID 2>/dev/null; sleep 2

rosservice call /finish_trajectory 0 > $OUT/finish.json 2>&1
sleep 5
rosservice call /write_state "filename: '$OUT/carto_smoke.pbstream'" > $OUT/write_state.json 2>&1
sleep 3

rosbag info $OUT/tracked_pose.bag > $OUT/baginfo.txt 2>&1
COUNT=$(grep -A2 tracked_pose $OUT/baginfo.txt | grep messages | head -1)
echo "RESULT: play_rc=$PLAY_RC tracked=$COUNT" > $OUT/RESULT.txt
tail -3 $OUT/carto_node.log >> $OUT/RESULT.txt
ls -la $OUT >> $OUT/RESULT.txt 2>&1

kill $NODE_PID 2>/dev/null
killall -q roscore rosmaster rosmaster 2>/dev/null
cat $OUT/RESULT.txt
