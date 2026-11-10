#!/bin/bash
# 语义滤除对比: FAST-LIO2 on kitti_0033_semfiltered.bag (复刻 54fd8cd2 流程)
set -x
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

rm -f /root/ws/src/FAST_LIO/Log/dbg.txt /root/ws/src/FAST_LIO/Log/fast_lio_time_log.csv \
      /root/ws/src/FAST_LIO/Log/mat_out.txt /root/ws/src/FAST_LIO/Log/pos_log.txt \
      /root/ws/src/FAST_LIO/PCD/scans.pcd

roscore > /tmp/roscore.log 2>&1 &
sleep 4
roslaunch fast_lio mapping_kitti.launch > /data/fastlio_semfiltered.log 2>&1 &
LAUNCH_PID=$!
sleep 10

rosbag record -O /data/kitti/odom_0033_semfiltered.bag /Odometry /path > /tmp/record.log 2>&1 &
REC=$!
sleep 3

rosbag play /data/kitti/kitti_0033_semfiltered.bag -r 0.5 2>&1 | tail -3
sleep 5

kill -INT $REC 2>/dev/null; sleep 3
kill -INT "$(pgrep -f fastlio_mapping | head -1)" 2>/dev/null
sleep 8
kill "$LAUNCH_PID" 2>/dev/null

# Odometry bag → timed traj csv
python3 - << 'PYEOF'
import rosbag, csv
bag = rosbag.Bag('/data/kitti/odom_0033_semfiltered.bag')
rows = []
for topic, msg, t in bag.read_messages(topics=['/Odometry']):
    p = msg.pose.pose.position
    rows.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
bag.close()
rows.sort()
with open('/data/kitti/odom_0033_semfiltered_timed.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'x', 'y', 'z'])
    w.writerows(rows)
print('timed traj 已存:', len(rows), '行')
PYEOF
cp /root/ws/src/FAST_LIO/Log/fast_lio_time_log.csv /data/kitti/odom_0033_semfiltered_time_log.csv 2>/dev/null
echo "SEMFILTERED RUN DONE"
