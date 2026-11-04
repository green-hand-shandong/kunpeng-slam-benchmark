#!/bin/bash
# KITTI + LeGO-LOAM 运行 + 轨迹录制（headless）
BAG="${BAG:?env BAG required}"; TAG="${TAG:?env TAG required}"
set -x
source /opt/ros/noetic/setup.bash
source /root/ws5/devel/setup.bash
roscore > /tmp/roscore.log 2>&1 &
sleep 4
roslaunch lego_loam run.launch > /tmp/lego.log 2>&1 &
LAUNCH=$!
sleep 10
rosbag record -O /data/kitti/lego_${TAG}.bag /aft_mapped_to_init > /tmp/record.log 2>&1 &
REC=$!
sleep 1
rosbag play --clock "$BAG" > /tmp/play.log 2>&1
sleep 3
kill -INT $REC 2>/dev/null; sleep 5
kill $LAUNCH 2>/dev/null
tail -4 /tmp/lego.log
python3 - <<'PYEOF'
import rosbag, math, os
try:
    bag = rosbag.Bag('/data/kitti/lego_%s.bag' % os.environ.get('TAG'))
    n = sum(1 for _ in bag.read_messages())
    print('recorded msgs:', n)
except Exception as e:
    print('bag err', e)
PYEOF
