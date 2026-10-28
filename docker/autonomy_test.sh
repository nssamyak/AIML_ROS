#!/usr/bin/env bash
# Headless end-to-end test: Gazebo + SLAM + Nav2 + frontier exploration.
# Writes logs, map snapshots and the final map to /out.
# Run from the repo root (takes ~5-10 min):
#   docker run --rm -v "$PWD/ros2_ws/src:/ros2_ws/src:ro" -v "$PWD/docker:/scripts:ro" -v "$PWD/docker/out:/out" hb_bot:jazzy bash /scripts/autonomy_test.sh
source /opt/ros/jazzy/setup.bash
mkdir -p /work && cp -r /ros2_ws/src /work/src && cd /work
colcon build > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
source install/setup.bash
rm -f /out/auto.log /out/map_*.pgm /out/map_*.yaml

LIMIT=${1:-1500}   # wall-clock seconds
ros2 launch hb_bringup sim_autonomy.launch.py headless:=true rviz:=false > /out/auto.log 2>&1 &
LAUNCH=$!
sleep 6
ros2 run ros_gz_bridge parameter_bridge '/world/arena/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'   --ros-args -r /world/arena/pose/info:=/gt_poses > /dev/null 2>&1 &
python3 /scripts/monitor.py > /out/monitor.csv 2>&1 &
start=$(date +%s)
n=0
while true; do
  sleep 30
  el=$(( $(date +%s) - start ))
  n=$((n+1))
  timeout 20 ros2 run nav2_map_server map_saver_cli -f /out/map_$(printf %03d $n) --ros-args -p use_sim_time:=true > /dev/null 2>&1
  goals=$(grep -c 'going to' /out/auto.log)
  echo "[$el s] goals sent: $goals  $(grep -E 'frontier_explorer' /out/auto.log | tail -1 | cut -c1-160)"
  if grep -q 'Done!' /out/auto.log; then echo "EXPLORATION FINISHED AND RETURNED HOME"; break; fi
  if [ $el -gt $LIMIT ]; then echo "TIME LIMIT REACHED"; break; fi
done
timeout 20 ros2 run nav2_map_server map_saver_cli -f /out/map_final --ros-args -p use_sim_time:=true > /dev/null 2>&1
echo "=== explorer log ==="
grep frontier_explorer /out/auto.log | sed 's/^.*\] //' | tail -60
echo "=== errors ==="
grep -E '\[ERROR\]|\[FATAL\]' /out/auto.log | sort | uniq -c | sort -rn | head -20
kill $LAUNCH; sleep 3
