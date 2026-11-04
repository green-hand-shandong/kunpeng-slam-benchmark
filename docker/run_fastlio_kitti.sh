#!/bin/bash
# KITTI + FAST-LIO 容器内一键运行脚本（headless）
# 用法: docker run --rm --net=host -v <ws>:/root/ws -v /data0/slam-datasets:/data \
#           -v <本目录>:/scripts fastlio:noetic bash /scripts/run_fastlio_kitti.sh
set -x
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

roscore > /tmp/roscore.log 2>&1 &
sleep 4

roslaunch fast_lio mapping_kitti.launch > /tmp/fastlio.log 2>&1 &
LAUNCH_PID=$!
sleep 10

# CPU/内存采样器（每 2s）
( while kill -0 "$LAUNCH_PID" 2>/dev/null && pgrep -f fastlio_mapping >/dev/null; do
    top -bn1 -p "$(pgrep -f fastlio_mapping | head -1)" 2>/dev/null | tail -1 \
      | awk '{print strftime("%H:%M:%S")" CPU="$9"% MEM="$10"%"}' >> /tmp/cpu.log
    sleep 2
done ) &
SAMPLER=$!

rosbag play /data/kitti/kitti_0001.bag 2>&1 | tail -4
sleep 3

kill -INT "$(pgrep -f fastlio_mapping | head -1)" 2>/dev/null   # 触发 PCD 保存
sleep 8
kill "$SAMPLER" "$LAUNCH_PID" 2>/dev/null

echo "===== CPU/MEM 采样 ====="
cat /tmp/cpu.log 2>/dev/null
echo "===== fastlio 日志尾部 ====="
tail -15 /tmp/fastlio.log
echo "===== 节点日志（关键行） ====="
cat /root/.ros/log/latest/laserMapping-1*.log 2>/dev/null | grep -viE "^$" | tail -25
echo "===== 节点 cwd 内容 ====="
ls -la /root/.ros/ 2>/dev/null | head
echo "===== PCD 产物 ====="
find / -name "*.pcd" -not -path "*/proc/*" 2>/dev/null | head -5
