# Workshop plan

Students go from a robot model to a real robot that explores a room by itself.
Each module ends with a checkpoint: something visible that proves it works
before moving on.

Suggested timing: **day 1** simulation (modules 0–7), **day 2** hardware (8–11).

**Before day 1 (optional, any laptop):** the [AI/ML modules](../aiml_modules/README.md)
explain the algorithms behind modules 5–7 (lidar, mapping, SLAM, planning,
MPPI, exploration, and a learned controller) in plain Python. They run on
Linux, macOS and Windows with no ROS install, so students can start before
their Ubuntu setup works.

---

## Module 0: Setup

**Ubuntu 24.04 + ROS 2 Jazzy** (native, or a VM with 3D acceleration):

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard
cd ~/Athena/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

**Windows/macOS:** use the Docker image in `docker/` (headless, or with an X
server / WSLg for the GUIs).

Checkpoint: `ros2 pkg list | grep hb_` shows five packages.

---

## Module 1: Describing the robot (URDF, xacro, TF)

```bash
ros2 launch hb_description display.launch.py
```

Concepts
* Links and joints. The TF tree: `base_footprint → base_link → rim_* → roller_*`,
  and `base_link → lidar_mount_link → lidar_link`.
* REP-103: x forward, y left, z up. The orange tab on the plate marks the front.
* xacro macros: `omni_wheel.xacro` builds a rim + 8 rollers from one angle.
  `lidar.xacro` builds the deck and the lidar.
* `sim_mode` switches between the Gazebo model and the real-robot model.

Exercises
1. Run `ros2 run tf2_tools view_frames` and find `lidar_link` in the PDF.
2. Raise the lidar by 3 cm (`standoff_height`). Rebuild, and check the TF in RViz.
3. Why do the rollers need joints at all? (Hint: what makes an omni wheel slide sideways?)

Checkpoint: the robot in RViz, wheels turning with the sliders.

---

## Module 2: Holonomic kinematics

Read `hb_control/hb_control/kinematics.py`. It's 30 lines of real code.

Each wheel can only push along its rolling direction (the tangent of the
circle it sits on). For wheel *i* at angle θᵢ:

```
r·ωᵢ = −sin θᵢ · vx + cos θᵢ · vy + L · ωz
```

Stack the three wheels to get `ω = J · [vx, vy, ωz]`. Invert J to get odometry.

Exercises
1. By hand: which wheels turn when the robot goes straight forward? Which one stays still? Why?
2. Run the tests: `python3 -m pytest src/hb_control/test -v`.
3. Write a test: pure rotation at 1 rad/s should spin every wheel at L/r rad/s.
   (It's already there. Find it and explain it.)
4. Bonus: a 4-wheel X-drive omni robot. Which line changes?

Checkpoint: all tests pass and you can explain J.

---

## Module 3: Simulation (Gazebo + ros2_control)

```bash
ros2 launch hb_gazebo sim.launch.py rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # hold Shift to strafe
```

Concepts
* `gz_ros2_control` makes the Gazebo joints look like real hardware.
  `wheel_velocity_controller` takes 3 wheel speeds; `joint_state_broadcaster` reports them.
* `ros_gz_bridge` passes `/clock` and `/scan` from Gazebo to ROS.
* `use_sim_time`: every node uses Gazebo's clock.
* The **same `kiwi_drive` node** will run on the real robot.

```bash
ros2 control list_controllers
ros2 topic echo /wheel_velocity_controller/commands
ros2 run hb_control wheel_test            # spins each wheel in turn
ros2 run hb_control wheel_test --motions  # forward, strafe, rotate
```

Checkpoint: the robot strafes sideways without turning.

---

## Module 4: Odometry

`kiwi_drive` integrates wheel angles into a pose in the `odom` frame.

Exercises
1. In RViz (fixed frame `odom`), enable the Odometry display. Drive a 1 m square with teleop.
   Does it close? Why not? (Rollers slip, and errors add up.)
2. Find where the midpoint heading is used in `integrate_pose`. Why is it there?
3. Change `wheel_radius` in `kiwi_drive.yaml` by 10% and see how far off odometry gets.

Checkpoint: you can explain why odometry alone can't build a map.

---

## Module 5: SLAM

```bash
ros2 launch hb_gazebo sim.launch.py
ros2 launch hb_navigation slam.launch.py use_sim_time:=true
ros2 launch hb_bringup rviz.launch.py use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Concepts
* slam_toolbox matches each scan against the map and publishes `map → odom`.
  That's the correction for odometry drift.
* Occupancy grid values: −1 unknown, 0 free, 100 occupied.
* Loop closure: drive a loop and watch the map snap into place.

Exercise: map the arena by hand and save it with
`ros2 run nav2_map_server map_saver_cli -f ~/maps/arena`. Open the `.pgm`.

Checkpoint: a clean map of all three rooms.

---

## Module 6: Navigation (Nav2)

```bash
ros2 launch hb_bringup sim_autonomy.launch.py explore:=false
```

Click **2D Goal Pose** in RViz.

Concepts (all in `hb_navigation/config/nav2_params.yaml`)
* Global costmap (from the map) and local costmap (rolling window from the lidar).
  `robot_radius` and the inflation layer.
* Planner (NavFn) finds a path. Controller (MPPI) follows it.
* **MPPI with `motion_model: "Omni"`:** every cycle it tries 1000 random
  velocity sequences (vx, vy, wz) over the next 2 s, scores each with
  *critics* (stay off obstacles, follow the path, reach the goal, don't spin),
  and blends the best. There is no PathAngle/GoalAngle critic because a
  holonomic robot doesn't need to face its path.
* Behaviour tree: replan, follow, and recover (clear costmaps, spin, back up, wait).
* `velocity_smoother` sits between the controller and `/cmd_vel`.

Exercises
1. Set `inflation_radius: 0.05` and send a goal through a doorway. Then 0.4. What changes?
2. Add `"PathAngleCritic"` to the MPPI critics (with `cost_weight: 2.0`,
   `threshold_to_consider: 0.5`, `offset_from_furthest: 4`, `max_angle_to_furthest: 1.0`).
   Now the robot turns to face its path. Is that better or worse for a holonomic robot?
3. Set `TwirlingCritic.twirling_cost_weight: 0.0`. What happens?
4. Lower `vx_max`/`vy_max` (MPPI) and the velocity_smoother limits to 0.08 m/s.

Checkpoint: the robot reaches a goal in another room.

---

## Module 7: Autonomous exploration

```bash
ros2 launch hb_bringup sim_autonomy.launch.py
```

Read `hb_navigation/hb_navigation/frontier_search.py` (the algorithm) and
`frontier_explorer.py` (the ROS node).

The algorithm
1. Frontier cell = free cell next to an unknown cell (`frontier_mask`).
2. Drop cells too close to walls; the robot can't park there.
3. Group touching cells into clusters (`cluster_cells`, a breadth-first search).
   Ignore tiny ones.
4. Score = distance − 0.5 × size. Pick the lowest. Skip spots already tried.
5. Send it to Nav2. Watch for: goal reached, failed, no progress for 15 s,
   or the frontier vanished because the lidar already saw it.
6. No frontiers 3 times in a row → done → drive home.

Exercises
1. Set `size_weight: 0.0` (pure nearest-first), then `3.0`. Time the full exploration for each.
2. What happens with `min_frontier_size: 0.05`? Why?
3. Edit `arena.sdf`: add a wall or a new room, and explore it.
4. Bonus: replace Euclidean distance with real path length (Nav2 `ComputePathToPose` action).

Checkpoint: the arena is fully mapped and the robot is back at the start.

---

## Module 8: Building the robot

[hardware.md](hardware.md) §2–4: parts, wiring, power. Common ground!

Checkpoint: the ESP32 powers up from USB and the motor driver from the battery,
with nothing hot or smoking.

## Module 9: Firmware and calibration

[hardware.md](hardware.md) §5–6. Flash the firmware, then do the calibration
steps in order: motor direction, encoder direction, counts/rev, feed-forward, PID.

Checkpoint: `V,8,8,8` holds all three wheels at about 8 rad/s.

## Module 10: Raspberry Pi bringup

[hardware.md](hardware.md) §7. Then:

```bash
ros2 launch hb_bringup robot.launch.py
ros2 run hb_control wheel_test --motions
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Checkpoint: teleop works, `/scan` shows up in RViz on the laptop, and the lidar points the right way.

## Module 11: Real autonomy

```bash
ros2 launch hb_bringup robot_autonomy.launch.py   # Pi
ros2 launch hb_bringup rviz.launch.py             # laptop
```

Build a small arena from cardboard (walls taller than the lidar!) and let it go.

Checkpoint: a saved map of the real arena.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Gazebo starts but no `/scan` | No GPU / OpenGL in the VM. Enable 3D acceleration, or try `export LIBGL_ALWAYS_SOFTWARE=1` |
| `wheel_velocity_controller` never activates | Gazebo didn't load `gz_ros2_control`. Install `ros-jazzy-gz-ros2-control`, check the sim log |
| Robot drives the wrong way in sim | You edited wheel angles in the URDF but not in `kiwi_drive.yaml` (or the reverse) |
| RViz: "No transform from map" | SLAM not running yet, or `use_sim_time` mismatch (all true in sim, all false on the robot) |
| Nav2: "goal is in collision" | Goal is too close to a wall. Raise the explorer's `clearance` |
| Explorer: "waiting for Nav2" forever | `lifecycle_manager_navigation` failed. Scroll up for the first error |
| Real robot spins in place on "forward" | Wheel order or motor direction wrong. Redo calibration step 1 and 6 |
| Real robot map smears when turning | Lidar `lidar_yaw` wrong, or odometry `wheel_base_radius` wrong |
| Explorer stops early | Map has small gaps behind obstacles. Lower `min_frontier_size` |
