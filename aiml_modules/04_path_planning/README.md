# Module 4 · Path planning (search)

**Question:** given a map, what's the best way from here to there?

## The idea

Planning is classic AI **search**: a grid of cells, moves between neighbours,
and a cost for every move. Find the cheapest sequence of moves.

Before searching we turn the map into a **costmap**:

* within one robot radius of a wall → **lethal** (the robot would touch it);
* a bit further out → **expensive** (the inflation layer), so paths keep
  a safe distance when there's room;
* open space → cost 1.

## The algorithms

**Dijkstra** grows outward from the start like a ripple, always expanding
the cell with the lowest cost-so-far `g`. Guaranteed cheapest, but it
explores in every direction, including away from the goal.

**A\*** expands the cell with the lowest `f = g + h`, where `h` is a guess
of the remaining cost (straight-line distance to the goal). As long as `h`
never *over*estimates (it's **admissible**), A\* still finds the cheapest
path, but it explores far fewer cells.

**Greedy A\*** (`f = g + 3·h`) trusts the guess too much. It's very fast,
but no longer guaranteed optimal.

```
Dijkstra:     f = g
A*:           f = g + h          h = straight-line distance (never too big)
greedy A*:    f = g + w·h, w > 1
```

## Where this lives in the real robot

| Here | Real robot (`nav2_params.yaml`) |
|---|---|
| `costmap()`: `robot_radius`, `inflation_radius`, `cost_scaling` | `global_costmap` → `robot_radius`, `inflation_layer` |
| `unknown_cost` (finite) | `allow_unknown: true`, used for exploration |
| `astar()` | `planner_server`, `GridBased` (NavFn, `use_astar`) |
| `plan(..., tolerance=0.3)` | NavFn `tolerance: 0.3` |

## Run it

```bash
python 04_path_planning/demo.py
python 04_path_planning/demo.py --animate
python 04_path_planning/demo.py --start 0.3 1.2 --goal 1.5 -1.1
python 04_path_planning/demo.py --inflation 0.1
```

## What to look for

* Colours show the order cells were expanded. Dijkstra floods; A\* heads for
  the goal; greedy A\* barely looks around.
* With the default goal all three find the same 3.96 m path, but the
  number of expanded cells is 1861 → 878 → 139.
* `--start 0.3 1.2 --goal 1.5 -1.1`: greedy A\* returns a **longer** path.
  Speed has a price.

## Exercises

1. Prove that straight-line distance never overestimates the true cost here.
   (Hint: what's the cheapest possible cell cost?)
2. Change the heuristic to Manhattan distance `|dx| + |dy|`. Is it still
   admissible with diagonal moves?
3. Use `--inflation 0.1` and `--inflation 0.4`. Which path would you want on
   a real robot with 2 cm odometry error?
4. Count expanded cells for 20 random start/goal pairs and plot Dijkstra vs A\*.
5. **Bonus:** Nav2 also offers Smac (hybrid A\*) and Theta\* planners. Read
   their docs. Why would a car-like robot need hybrid A\* but our holonomic
   robot doesn't?
