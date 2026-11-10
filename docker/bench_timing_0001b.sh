#!/bin/bash
set -u
D=/data2/gxw/slam/docker
OUT=/data0/slam-datasets/results/bench_0001
run_one() {
  local algo=$1 img=$2 scr=$3
  echo "[$(date +%H:%M:%S)] bench $algo"
  local t0=$(date +%s.%N)
  docker run --rm --net=host -e BAG=/data/kitti/kitti_0001.bag -e TAG=$algo -e PLAY_RATE=1.0 \
    -v /data2/gxw/slam/ws:/root/ws -v /data2/gxw/slam/ws3:/root/ws3 \
    -v /data2/gxw/slam/ws4:/root/ws4 -v /data2/gxw/slam/ws5:/root/ws5 \
    -v /data0/slam-datasets:/data -v $D:/scripts:ro \
    $img bash $scr > $OUT/${algo}.log 2>&1
  local rc=$?
  local t1=$(date +%s.%N)
  echo "{\"algo\": \"$algo\", \"wall_s\": $(echo "$t1 - $t0" | bc), \"rc\": $rc, \"ts\": \"$(date -Iseconds)\"}" > $OUT/${algo}.json
  echo "  → wall=$(echo "$t1 - $t0" | bc)s rc=$rc"
}
run_one pointlio fastlio:noetic  "/scripts/run_pointlio_kitti.sh"
run_one liosam   lio-sam:noetic  "/scripts/run_liosam_kitti.sh"
echo "BENCH B DONE"
