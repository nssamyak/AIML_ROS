# Module 1 · Lidar perception

**Question:** what does the robot actually know about the world?

## The idea

A 2D lidar spins a laser and measures, for 360 directions, how far away the
first obstacle is. That's all: **360 numbers per scan**, ten times a second.
No colours, no object names, and nothing behind the first thing each beam hits.

Everything the robot later "understands" (maps, paths, decisions) is built
from these numbers.

## The maths

A scan is a list of `(angle, range)` pairs: **polar** coordinates.
To use them we convert to points:

```
robot frame (x forward, y left):    x = r·cos(a)        y = r·sin(a)

world frame (robot at X, Y facing θ):
    x_w = X + x·cos θ − y·sin θ
    y_w = Y + x·sin θ + y·cos θ
```

The second step needs the robot's pose. If the pose is wrong, every point is
wrong. Remember this for module 3.

Real sensors are noisy. Our model adds Gaussian noise with σ = 1 cm, and
returns "no reading" (`inf`) for anything closer than 12 cm or further than 8 m.

## Where this lives in the real robot

| Here | Real robot |
|---|---|
| `hbai/lidar.py` `Lidar.scan()` | `rplidar_node` publishing `/scan` (`sensor_msgs/LaserScan`) |
| `scan_to_points(..., pose)` | TF transform `lidar_link → map` |
| `hbai/world.py` `raycast()` | Gazebo's `gpu_lidar` sensor in simulation |

## Run it

```bash
python 01_lidar_perception/demo.py
python 01_lidar_perception/demo.py --pose 0.5 0.8 1.57
python 01_lidar_perception/demo.py --animate
```

## What to look for

* **(a)** Walls are only seen where a beam reaches them. Obstacles cast
  "shadows" the robot knows nothing about.
* **(b)** The raw data. This is literally what `ros2 topic echo /scan` shows.
* **(c)** The same points relative to the robot. This view doesn't change
  when the robot moves through the world; panel (a) does.
* **(d)** The noise: measure the same wall 300 times and the readings spread
  by about ±1 cm.

## Exercises

1. Turn the robot to face up: `--pose -1.3 -0.2 1.57`. Panel (c) rotates,
   panel (a) doesn't change. Explain why.
2. Set `--noise 0.05`. How would that hurt mapping later?
3. Find a pose where a whole room is in shadow.
4. In `hbai/lidar.py` change `n_beams` to 36. What can't the robot see any more?
5. **Bonus:** the real RPLidar is mounted 0.12 m above the floor. Which
   obstacles in a real room would it miss completely?
