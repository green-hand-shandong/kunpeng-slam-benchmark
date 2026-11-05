#!/bin/bash
# Cartographer 2D offline：deutsches_museum bag → pbstream + occupancy grid
set -x
source /opt/ros/noetic/setup.bash
source /ws2/devel_carto/setup.bash
BAG="${BAG:?env BAG required}"
OUT="${OUT:-/data/modality_A}"
roscore > /tmp/roscore2d.log 2>&1 &
sleep 4
timeout 600 /ws2/devel_carto/lib/cartographer_ros/cartographer_offline_node \
  -configuration_directory /ws2/cfg -configuration_basenames backpack_2d.lua \
  -bag_filenames "$BAG" > /tmp/carto2d.log 2>&1
echo OFFLINE_EXIT=$?
grep -E "Elapsed wall|Elapsed CPU" /tmp/carto2d.log | tail -2
ls -la "$BAG.pbstream" 2>/dev/null
pkill -f roscore 2>/dev/null
