# Module 3 · SLAM: Simultaneous Localisation And Mapping

**Question:** module 2 cheated by knowing the true pose. What if the robot
has to work out where it is from its own wheels?

## The idea

**Odometry** (counting wheel rotations) tells you how far you moved. But omni
wheels slip, wheel sizes are never exact, and the small errors **add up**.
After a few metres the robot is confidently wrong, and a map built with
those poses smears and doubles its walls.

The fix: **compare each new scan with the map built so far.** If odometry
says you're at X but the scan lines up with the walls best at X + 3 cm,
believe the scan. That's *scan matching*, and it's the heart of SLAM.

The chicken-and-egg problem is that you need the map to localise and the pose
to map. SLAM does both at once: correct the pose with the map, then extend
the map with the corrected pose.

## The maths: correlative scan matching

1. **Predict:** `guess = previous pose ⊕ odometry motion`.
2. **Likelihood field:** blur the occupied cells with a Gaussian, so being
   2 cm off a wall still scores well and the search has a smooth landscape.
3. **Search:** try every pose in a grid around the guess (±15 cm, ±8.6°),
   coarse then fine (~3,500 poses). For each, transform the scan points and
   average the field value under them.
4. **Pick** the best score, minus a small penalty for moving far from the
   guess (odometry is wrong, but not wildly wrong).

```
score(pose) = mean_k field( pose ⊕ point_k ) − λ_xy·|Δxy|² − λ_θ·|Δθ|²
```

No learning involved: it's search plus a good scoring function.

What's not here (but is in `slam_toolbox`): **loop closure**. Recognising a
place you've visited before and spreading the accumulated error back along
the whole trajectory with a pose-graph optimiser.

## Where this lives in the real robot

| Here | Real robot (`slam_toolbox.yaml`) |
|---|---|
| search ±0.15 m, step 0.025 then 0.005 | `correlation_search_space_dimension`, `..._resolution` |
| blur `sigma=0.05` | `correlation_search_space_smear_deviation` |
| `penalty_xy`, `penalty_th` | `distance_variance_penalty`, `angle_variance_penalty` |
| `DriftingOdometry` | `kiwi_drive` → `/odom` (real slip!) |
| corrected pose | the `map → odom` transform slam_toolbox publishes |

## Run it

```bash
python 03_slam_scan_matching/demo.py
python 03_slam_scan_matching/demo.py --yaw-drift 0.15
python 03_slam_scan_matching/demo.py --no-penalty
```

## What to look for

* **Left:** odometry-only map. Walls bend and appear twice; the magenta path
  wanders off the green one.
* **Middle:** same wheels, same scans, plus scan matching. Clean walls.
* **Right:** odometry error keeps growing (tens of cm after 12 m); the
  scan-matched error stays at a few cm.

## Exercises

1. Raise `--yaw-drift` until scan matching fails too. What's the limit, and
   which search parameter would you change to survive it?
2. `--match-every 10`: correct less often. Why does it get worse?
3. In `scan_matching.py`, make the search window ±5 cm. Explain what breaks.
4. Scan matching fails in a long featureless corridor. Why? (Think about
   which direction along the corridor is "best".)
5. **Bonus:** on the real robot, open RViz with fixed frame `odom` and then
   `map`, and drive a loop. Watch the `map → odom` transform jump: that's
   this correction happening live.
