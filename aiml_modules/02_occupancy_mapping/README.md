# Module 2 · Occupancy grid mapping

**Question:** how do noisy scans turn into a reliable map?

## The idea

Cut the floor into 5 cm squares (cells). For every cell, keep a **belief**:
the probability that the cell is occupied. Start at 50% ("no idea").

Every lidar beam is evidence:

* cells the beam flew **through** were probably free, so the belief goes down;
* the cell where the beam **stopped** is probably occupied, so the belief goes up.

One reading can be wrong, but hundreds of readings agreeing can't. The map
is the accumulated vote of all scans.

## The maths (a Bayes filter per cell)

Bayes' rule for a yes/no question becomes simple addition in **log-odds**:

```
l = log( p / (1 − p) )            p = 0.5  ↔  l = 0

l_new = l_old + l_measurement     l_occ = +0.85 (hit)   l_free = −0.4 (pass-through)

p = 1 / (1 + e^(−l))              to read the belief back as a probability
```

So mapping is just counting evidence. Values are clipped to ±4 so a cell can
still change its mind if something moves.

For ROS, the map is published with three values (what `slam_toolbox` sends):
`−1` unknown, `0` free, `100` occupied.

## Where this lives in the real robot

| Here | Real robot |
|---|---|
| `hbai/mapping.py` `OccupancyGrid.update()` | inside `slam_toolbox` |
| `to_ros()` | the `/map` topic (`nav_msgs/OccupancyGrid`) |
| `--res 0.05` | `resolution: 0.05` in `hb_navigation/config/slam_toolbox.yaml` |

## Run it

```bash
python 02_occupancy_mapping/demo.py
python 02_occupancy_mapping/demo.py --animate
python 02_occupancy_mapping/demo.py --l-free -0.1 --l-occ 2.0
```

## What to look for

* The snapshots: grey (unknown) shrinks as the robot drives.
* The insides of obstacles stay grey. No beam ever gets in there.
* Bottom right: a wall cell and a free cell converge to 1 and 0 within a
  few scans. Each scan is one Bayes update.

## Exercises

1. `--l-free -0.05`: free space now needs many more scans. Why might you
   *want* that? (Hint: moving people.)
2. `--res 0.02` and `--res 0.15`: what do you gain and lose? Think about
   memory and the Raspberry Pi.
3. `--noise 0.05`: walls get thicker. Why?
4. Write a function that counts how many cells changed from unknown to known
   in each scan. When does the robot learn the most?
5. **Bonus:** the clipping at ±4 is a design choice. What happens to a door
   that opens after the robot has mapped it as a wall, with and without clipping?
