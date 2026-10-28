# Module 6 · Autonomous exploration (decision making)

**Question:** nobody gives the robot a goal. How does it decide where to go?

## The idea

A **frontier** is the border between known free space and unknown space.
Standing at a frontier lets the lidar see past it. So:

```
repeat:
    sense   → lidar scan
    map     → update the occupancy grid            (module 2, or 3 with --slam)
    decide  → find frontiers, pick the best one
    plan    → A* path to it                          (module 4)
    act     → follow the path with MPPI              (module 5)
until no frontiers are left → drive home
```

This is a complete **agent**: it perceives, keeps a model of the world (the
map), chooses goals that maximise information, and acts.

## How a frontier is chosen

1. **Detect:** free cells that touch unknown cells (two array shifts in numpy).
2. **Filter:** drop cells too close to walls (the robot can't park there).
3. **Cluster:** group touching cells (breadth-first search); drop tiny clusters.
4. **Score:** `cost = distance − 0.5 × size`. Near and long frontiers win.
5. **Commit, but stay alert.** Every second, re-check:
   * frontier gone (the lidar already saw past it) → pick the next one;
   * no progress for 15 s → blacklist it, pick the next;
   * path blocked on the updated map → blacklist it.

This is a **greedy** strategy: it's not guaranteed to find the shortest
exploration route (that problem is NP-hard), but it's simple and robust.

## Where this lives in the real robot

| Here | Real robot |
|---|---|
| `hbai/frontiers.py` | `hb_navigation/frontier_search.py` (**identical code**) |
| `Explorer.decide()` / `choose_frontier()` | `hb_navigation/frontier_explorer.py` |
| `ExploreConfig` | `hb_navigation/config/explorer.yaml` |
| `plan()` + `MPPI` | Nav2 `NavigateToPose` action |

## Run it

```bash
python 06_frontier_exploration/demo.py
python 06_frontier_exploration/demo.py --slam
python 06_frontier_exploration/demo.py --controller simple
python 06_frontier_exploration/demo.py --size-weight 3
```

## What to look for

* Cyan frontiers and orange candidates appear, and the robot heads for one
  (green star). The goal often switches *before* the robot arrives: the lidar
  already saw past it.
* The coverage curve: steep at first, then a long flat tail. The last few
  pockets (behind the cylinder, between the box and the wall) cost more time
  than the first 90%.
* With `--slam` the magenta robot is where it *thinks* it is; it stays on
  top of the blue (true) one.

## Exercises

1. Time the full exploration for `--size-weight 0`, `0.5`, `3`. Which wins, and why?
2. `--min-frontier-size 0.5`: much faster, but what does the map miss?
   When is that an acceptable trade?
3. `--controller simple` ignores obstacles. Why does it still work here? Build
   a world (edit `hbai/world.py`) where it fails.
4. Add a "return home when battery < 20%" rule. Assume the battery drains
   1% per 10 s of driving.
5. **Bonus:** replace straight-line distance in the score with the A\* path
   length. Is the extra planning time worth it?
