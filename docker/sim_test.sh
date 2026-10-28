#!/usr/bin/env bash
# Run from the repo root:
#   docker run --rm -v "$PWD/ros2_ws/src:/ros2_ws/src:ro" -v "$PWD/docker:/scripts:ro" -v "$PWD/docker/out:/out" hb_bot:jazzy bash /scripts/sim_test.sh
# Headless Gazebo test: controllers, lidar, and odometry vs ground truth.
set -o pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work && cp -r /ros2_ws/src /work/src && cd /work
colcon build --event-handlers console_direct- > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
source install/setup.bash

ros2 launch hb_gazebo sim.launch.py headless:=true > /out/sim.log 2>&1 &
for i in $(seq 1 90); do
  ros2 control list_controllers 2>/dev/null | grep -q 'wheel_velocity_controller.*active' && break
  sleep 2
done
echo "=== controllers ==="; ros2 control list_controllers
echo "=== topics ==="; ros2 topic list
echo "=== /scan ==="; timeout 15 ros2 topic hz /scan --window 10 2>&1 | head -4
timeout 10 ros2 topic echo /scan --once --field ranges 2>/dev/null | tr ',' '\n' | grep -vc inf | xargs echo "finite ranges in one scan:"
echo "=== /odom ==="; timeout 10 ros2 topic hz /odom --window 20 2>&1 | head -2

ros2 run ros_gz_bridge parameter_bridge '/world/arena/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  --ros-args -r /world/arena/pose/info:=/gt_poses > /dev/null 2>&1 &
sleep 3
echo "=== drive test ==="
python3 /scripts/drive_test.py
echo "=== sim log errors ==="
grep -iE 'error|exception' /out/sim.log | grep -v 'Error: No' | head -20
