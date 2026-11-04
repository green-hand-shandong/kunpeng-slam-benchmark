#!/bin/bash
# KITTI + FAST-LIO 运行 + 里程计录制 + 轨迹分析（headless 验证版）
# 必须由调用方显式传入 BAG/TAG，缺失立即报错退出（防止静默跑错数据集）
BAG="${BAG:?env BAG is required (bag path inside container)}"
TAG="${TAG:?env TAG is required}"
SAVE_WAIT="${SAVE_WAIT:-8}"
PLAY_RATE="${PLAY_RATE:-1.0}"
set -x
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

roscore > /tmp/roscore.log 2>&1 &
sleep 4
# 预清理上一次运行的产物（只清文件保留目录：FAST-LIO 的 fopen 不建目录，删目录会段错误）
rm -f /root/ws/src/FAST_LIO/Log/* /root/ws/src/FAST_LIO/PCD/* 2>/dev/null
mkdir -p /root/ws/src/FAST_LIO/Log /root/ws/src/FAST_LIO/PCD
roslaunch fast_lio mapping_kitti.launch > /tmp/fastlio.log 2>&1 &
LAUNCH_PID=$!
sleep 10

rosbag record -O /data/kitti/odom_${TAG}.bag /Odometry /path > /tmp/record.log 2>&1 &
REC=$!
sleep 1

rosbag play "$BAG" -r "$PLAY_RATE" > /tmp/play.log 2>&1
sleep 3

kill -INT "$(pgrep -f fastlio_mapping | head -1)" 2>/dev/null
sleep "$SAVE_WAIT"
kill "$REC" 2>/dev/null
sleep 2
kill "$LAUNCH_PID" 2>/dev/null

echo "===== fastlio.log tail ====="
tail -6 /tmp/fastlio.log 2>/dev/null
echo "===== play.log tail ====="
tail -c 600 /tmp/play.log 2>/dev/null | tr '\r' '\n' | tail -3
echo "===== record.log ====="
tail -5 /tmp/record.log 2>/dev/null
echo "===== roscore.log ====="
tail -3 /tmp/roscore.log 2>/dev/null
echo "===== 轨迹分析 ====="
python3 - <<'EOF'
import rosbag, math
import os
bag = rosbag.Bag(os.environ.get("BAGOUT", "/data/kitti/odom_%s.bag" % os.environ.get("TAG", "0001")))
poses = []
for t, m, _ in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position
    poses.append((p.x, p.y, p.z))
bag.close()
n = len(poses)
dist = sum(math.dist(poses[i], poses[i+1]) for i in range(n-1))
start, end = poses[0], poses[-1]
drift = math.dist(start, end)
print(f"odom 消息数: {n}")
print(f"轨迹总长: {dist:.1f} m")
print(f"起点: ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})")
print(f"终点: ({end[0]:.2f}, {end[1]:.2f}, {end[2]:.2f})")
print(f"起终点直线距离(短序列无闭环, 仅供观察): {drift:.1f} m")
with open('/data/kitti/odom_%s_traj.csv' % os.environ.get("TAG", "0001"), 'w') as f:
    f.write("x,y,z\n")
    for p in poses:
        f.write(f"{p[0]},{p[1]},{p[2]}\n")
print("轨迹 CSV 已存: /data/kitti/odom_%s_traj.csv" % os.environ.get("TAG", "0001"))
# 带时间戳轨迹（供 ATE/RPE 时间关联）；注意 rosbag 元组顺序是 (topic, msg, time)
bag = rosbag.Bag(os.environ.get("BAGOUT", "/data/kitti/odom_%s.bag" % os.environ.get("TAG", "0001")))
with open('/data/kitti/odom_%s_timed.csv' % os.environ.get("TAG", "0001"), 'w') as f:
    f.write('t,x,y,z\n')
    for topic, m, tt in bag.read_messages(topics=['/Odometry']):
        p = m.pose.pose.position
        f.write(f'{m.header.stamp.to_sec():.3f},{p.x:.3f},{p.y:.3f},{p.z:.3f}\n')
bag.close()
print("带时间戳轨迹已存")
EOF
echo "===== PCD ====="
ls -la /root/ws/src/FAST_LIO/PCD/ 2>/dev/null
python3 - <<'EOF'
import numpy as np
p = '/root/ws/src/FAST_LIO/PCD/scans.pcd'
with open(p, 'rb') as f:
    header = {}
    while True:
        line = f.readline().decode(errors='ignore').strip()
        if line.startswith('POINTS'): header['points'] = int(line.split()[1])
        if line.startswith('DATA'): break
    print("PCD 点数:", header.get('points'))
EOF