#!/bin/bash
# VINS-Fusion EuRoC MH_01_easy 运行脚本
# 用法：在 yolo26_train 容器内（有 torch_npu 不需要，只用 CPU ROS）
set -x
source /opt/ros/noetic/setup.bash
source /root/ws6/devel/setup.bash

BAG="${BAG:?env BAG required}"
CONFIG="${CONFIG:-/root/ws6/config_euroc/euroc_stereo_imu_config.yaml}"

roscore > /tmp/roscore_vins.log 2>&1 &
sleep 4

# VINS estimator（双目+IMU）
rosrun vins vins_node "$CONFIG" > /tmp/vins_euroc.log 2>&1 &
VINS_PID=$!
sleep 8

# loop_fusion（可选，回环）
rosrun loop_fusion loop_fusion_node "$CONFIG" > /tmp/loopfusion.log 2>&1 &

# 录制 odometry
rosbag record -O /data/euroc_vins_odometry.bag /lio_sam/mapping/odometry_incremental > /tmp/record_vins.log 2>&1 &
REC=$!
# VINS 的 odometry 话题：
# 先查实际话题名
sleep 3
TOPICS=$(rostopic list 2>/dev/null | grep -iE "odom|path" | head -4)
echo "topics: $TOPICS"
pkill -f "rosbag record" 2>/dev/null; sleep 1

# 用正确话题录制
ODO_TOPIC=$(rostopic list 2>/dev/null | grep -i "odometry" | grep -v imu | head -1)
echo "recording: $ODO_TOPIC"
rosbag record -O /data/euroc_vins_odometry.bag "$ODO_TOPIC" > /tmp/record_vins.log 2>&1 &
REC=$!
sleep 2

# 回放 EuRoC bag
timeout 300 rosbag play --clock "$BAG" > /tmp/play_vins.log 2>&1
sleep 5

kill -INT $REC 2>/dev/null; sleep 4
kill $VINS_PID 2>/dev/null

# 评估
python3 << 'PYEOF'
import rosbag, math
bag = rosbag.Bag('/data/euroc_vins_odometry.bag')
rows = []
for topic, msg, t in bag.read_messages():
    p = msg.pose.pose.position
    rows.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
bag.close()
rows.sort()
with open('/data/euroc_vins_traj.csv', 'w') as f:
    f.write('t,x,y,z\n')
    for r in rows: f.write('%.6f,%.6f,%.6f,%.6f\n' % r)
xs=[r[1] for r in rows]; ys=[r[2] for r in rows]
L = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]) for i in range(1,len(xs)))
print('poses:', len(rows), '| 2D path: %.1f m' % L)
PYEOF
