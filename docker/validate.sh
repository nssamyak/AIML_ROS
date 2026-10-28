#!/usr/bin/env bash
# Static checks: build, URDF, geometry, unit tests. Run from the repo root:
#   docker run --rm -v "$PWD/ros2_ws/src:/ros2_ws/src:ro" -v "$PWD/docker:/scripts:ro" hb_bot:jazzy bash /scripts/validate.sh
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work && cp -r /ros2_ws/src /work/src && cd /work

echo "=== colcon build ==="
colcon build --event-handlers console_direct- 2>&1 | tail -5
source install/setup.bash

XACRO=$(ros2 pkg prefix hb_description)/share/hb_description/urdf/hb_bot.urdf.xacro
echo "=== xacro + check_urdf (sim) ==="
xacro "$XACRO" sim_mode:=true > /tmp/sim.urdf
check_urdf /tmp/sim.urdf > /tmp/cu1.txt && head -3 /tmp/cu1.txt
echo "=== xacro + check_urdf (real robot) ==="
xacro "$XACRO" sim_mode:=false > /tmp/real.urdf
check_urdf /tmp/real.urdf > /tmp/cu2.txt && head -3 /tmp/cu2.txt
echo "links: $(grep -c '<link ' /tmp/sim.urdf), continuous joints sim: $(grep -c 'type=\"continuous\"' /tmp/sim.urdf), real: $(grep -c 'type=\"continuous\"' /tmp/real.urdf)"

echo "=== URDF -> SDF (what Gazebo loads) ==="
gz sdf -p /tmp/sim.urdf > /tmp/sim.sdf && echo "sdf ok, sensors: $(grep -c '<sensor' /tmp/sim.sdf)"

echo "=== wheel geometry vs kinematics ==="
python3 src/hb_description/test/check_wheel_geometry.py /tmp/sim.urdf

echo "=== unit tests ==="
python3 -m pytest -q src/hb_control/test src/hb_navigation/test 2>&1 | tail -3

echo "=== launch files parse ==="
for f in $(find install -name '*.launch.py'); do
  python3 -c "import importlib.util,sys; s=importlib.util.spec_from_file_location('m','$f'); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); m.generate_launch_description()" \
    && echo "ok  $(basename $f)"
done
echo "ALL STATIC CHECKS PASSED"
