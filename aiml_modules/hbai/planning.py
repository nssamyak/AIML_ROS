"""Global path planning on a costmap: Dijkstra and A*.

This is what Nav2's planner_server does (NavFn plugin). Two steps:

1. COSTMAP. Turn the map into travel costs.
     - lethal (can't go) within the robot radius of a wall
     - expensive close to walls, cheaper further away (inflation layer)
     - cost 1 in open space
2. SEARCH. Find the cheapest path over the grid.
     Dijkstra expands cells in order of cost-so-far g.
     A* expands in order of g + h, where h (straight-line distance to the
     goal) is a guess of the remaining cost. Same path, far fewer cells.
"""
import heapq
import math

import numpy as np

LETHAL = np.inf


def distance_field(occupied, res, max_dist):
    """Distance (m) from every cell to the nearest occupied cell, capped at max_dist."""
    h, w = occupied.shape
    r = int(math.ceil(max_dist / res))
    dist = np.full(occupied.shape, float(max_dist))
    dist[occupied] = 0.0
    padded = np.pad(occupied, r)
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            d = math.hypot(dr, dc) * res
            if d == 0 or d > max_dist:
                continue
            shifted = padded[r + dr:r + dr + h, r + dc:r + dc + w]
            np.minimum(dist, np.where(shifted, d, max_dist), out=dist)
    return dist


def costmap(occupied, res, unknown=None, robot_radius=0.08, inflation_radius=0.25,
            cost_scaling=5.0, unknown_cost=2.0):
    """Travel cost per cell: LETHAL near walls, decaying cost around them, 1 in free space.

    unknown cells get `unknown_cost` (finite) so paths may cross unexplored space,
    like Nav2's allow_unknown: true.
    """
    d = distance_field(occupied, res, inflation_radius + res)
    cost = np.ones(occupied.shape)
    band = (d > robot_radius) & (d < inflation_radius)
    cost[band] += 50.0 * np.exp(-cost_scaling * (d[band] - robot_radius))   # Nav2's inflation formula
    if unknown is not None:
        cost[unknown] = np.maximum(cost[unknown], unknown_cost)
    cost[d <= robot_radius] = LETHAL
    return cost


NEIGHBOURS = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)), (0, -1, 1.0),
              (0, 1, 1.0), (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]


def astar(cost, start, goal, heuristic_weight=1.0):
    """Cheapest 8-connected path from start to goal (row, col).

    heuristic_weight = 0 -> Dijkstra, 1 -> A*, > 1 -> greedy (faster, not optimal).
    Returns (path as list of cells or None, list of expanded cells in order).
    """
    h, w = cost.shape
    gr, gc = goal
    g = np.full(cost.shape, np.inf)
    parent = {}
    closed = np.zeros(cost.shape, dtype=bool)
    g[start] = 0.0
    heap = [(heuristic_weight * math.hypot(start[0] - gr, start[1] - gc), 0.0, start)]
    expanded = []

    while heap:
        _, g_here, cell = heapq.heappop(heap)
        if closed[cell]:
            continue
        closed[cell] = True
        expanded.append(cell)
        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return path[::-1], expanded
        r, c = cell
        for dr, dc, step in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or closed[nr, nc] or cost[nr, nc] == LETHAL:
                continue
            new_g = g_here + step * 0.5 * (cost[r, c] + cost[nr, nc])
            if new_g < g[nr, nc]:
                g[nr, nc] = new_g
                parent[(nr, nc)] = cell
                f = new_g + heuristic_weight * math.hypot(nr - gr, nc - gc)
                heapq.heappush(heap, (f, new_g, (nr, nc)))
    return None, expanded


def plan(geom, cost, start_xy, goal_xy, heuristic_weight=1.0, tolerance=0.3):
    """Plan in metres. Returns (path (N, 2) or None, expanded cells).

    If the goal cell is lethal, the closest free cell within `tolerance` is used
    instead (like NavFn's tolerance). The start cell is always allowed, so a
    robot that drifted into the inflation zone can still get out.
    """
    cost = cost.copy()
    start = tuple(int(v) for v in geom.to_cell(np.asarray(start_xy)[:2]))
    goal = tuple(int(v) for v in geom.to_cell(np.asarray(goal_xy)[:2]))
    if not (geom.in_bounds(*start) and geom.in_bounds(*goal)):
        return None, []
    cost[start] = min(cost[start], 1.0) if np.isfinite(cost[start]) else 1.0

    if cost[goal] == LETHAL:
        r = int(tolerance / geom.res)
        rows, cols = np.mgrid[goal[0] - r:goal[0] + r + 1, goal[1] - r:goal[1] + r + 1]
        ok = geom.in_bounds(rows, cols)
        rows, cols = rows[ok], cols[ok]
        ok = np.isfinite(cost[rows, cols]) & (np.hypot(rows - goal[0], cols - goal[1]) <= r)
        if not ok.any():
            return None, []
        i = np.argmin(np.where(ok, np.hypot(rows - goal[0], cols - goal[1]), np.inf))
        goal = (int(rows[i]), int(cols[i]))

    cells, expanded = astar(cost, start, goal, heuristic_weight)
    if cells is None:
        return None, expanded
    rc = np.array(cells)
    return geom.to_world(rc[:, 0], rc[:, 1]), expanded


def path_length(path):
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))) if path is not None and len(path) > 1 else 0.0
