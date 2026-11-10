#!/bin/bash
# KITTI + Point-LIO 运行 + 里程计录制（headless 验证版，仿 run_fastlio_kitti_verify.sh）
BAG="${BAG:?env BAG is required (bag path inside container)}"
TAG="${TAG:?env TAG is required}"
SAVE_WAIT="${SAVE_WAIT:-8}"
PLAY_RATE="${PLAY_RATE:-1.0}"
set -x
source /opt/ros/noetic/setup.bash
source /root/ws3/devel/setup.bash

roscore > /tmp/roscore.log 2>&1 &
sleep 4
rm -f /root/ws3/src/Point-LIO/Log/* 2>/dev/null
mkdir -p /root/ws3/src/Point-LIO/Log /root/ws3/src/Point-LIO/PCD

roslaunch point_lio mapping_kitti.launch > /tmp/pointlio.log 2>&1 &
LAUNCH_PID=$!
sleep 10

rosbag record -O /data/kitti/odom_${TAG}.bag /aft_mapped_to_init > /tmp/record.log 2>&1 &
REC=$!
sleep 1

rosbag play "$BAG" -r "$PLAY_RATE" > /tmp/play.log 2>&1
sleep 3

kill -INT "$(pgrep -f pointlio_mapping | head -1)" 2>/dev/null
sleep "$SAVE_WAIT"
kill "$REC" 2>/dev/null
sleep 2
kill "$LAUNCH_PID" 2>/dev/null

echo "===== pointlio.log tail ====="
tail -8 /tmp/pointlio.log 2>/dev/null
echo "===== record.log ====="
tail -4 /tmp/record.log 2>/dev/null
echo "===== 轨迹分析 ====="
python3 - <<'PYEOF'
import rosbag, math, os
bag = rosbag.Bag("/data/kitti/odom_%s.bag" % os.environ.get("TAG", "0001"))
poses = []
for t, m, _ in bag.read_messages(topics=['/Odometry']):
    p = m.pose.pose.position
    poses.append((p.x, p.y, p.z))
bag.close()
n = len(poses)
if n > 1:
    dist = sum(math.dist(poses[i], poses[i+1]) for i in range(n-1))
    print(f"odom 消息数: {n}")
    print(f"轨迹总长: {dist:.1f} m")
    print(f"终点: ({poses[-1][0]:.2f}, {poses[-1][1]:.2f}, {poses[-1][2]:.2f})")
else:
    print("no odometry messages!")
PYEOF
