"""Frontier detection on an occupancy grid. Pure numpy, no ROS.

A FRONTIER is a free cell that touches an unknown cell: the edge of what
the robot has mapped so far. Driving to a frontier lets the lidar see past
it, so repeatedly visiting frontiers maps the whole area.

Occupancy grid values (nav_msgs/OccupancyGrid):
    -1        unknown
    0..100    probability of being occupied (SLAM uses 0 = free, 100 = wall)
"""
from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Frontier:
    x: float                 # goal point (a frontier cell near the cluster centre), metres
    y: float
    size: float              # length of the frontier, metres
    cells: np.ndarray = field(repr=False, default=None)   # (N, 2) world x, y of every cell


def dilate(mask, radius):
    """Grow True regions of a 2D bool array by `radius` cells (square kernel)."""
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    for k in range(1, radius + 1):       # rows
        out[k:, :] |= mask[:-k, :]
        out[:-k, :] |= mask[k:, :]
    rows = out.copy()
    for k in range(1, radius + 1):       # then columns
        out[:, k:] |= rows[:, :-k]
        out[:, :-k] |= rows[:, k:]
    return out


def frontier_mask(grid, clearance_cells=0, free_threshold=25, occupied_threshold=65):
    """Bool array marking frontier cells. `grid` is a (height, width) int array."""
    unknown = grid < 0
    free = (grid >= 0) & (grid < free_threshold)
    occupied = grid >= occupied_threshold

    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]

    frontier = free & touches_unknown
    if clearance_cells > 0:
        # The robot can't reach a spot that is right next to a wall
        frontier &= ~dilate(occupied, clearance_cells)
    return frontier


def cluster_cells(mask):
    """Group touching True cells (8-connected). Returns a list of (N, 2) [row, col] arrays."""
    remaining = set(map(tuple, np.argwhere(mask)))
    clusters = []
    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        members = [seed]
        while queue:
            r, c = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (r + dr, c + dc)
                    if n in remaining:
                        remaining.remove(n)
                        queue.append(n)
                        members.append(n)
        clusters.append(np.array(members))
    return clusters


def find_frontiers(grid, resolution, origin_x, origin_y,
                   clearance=0.1, min_size=0.25,
                   free_threshold=25, occupied_threshold=65):
    """Find frontier clusters in a map.

    grid         (height, width) array of occupancy values, row 0 = bottom of the map
    resolution   metres per cell
    origin_x/y   world position of cell (0, 0)'s corner
    clearance    keep goals at least this far from walls (metres)
    min_size     ignore frontiers shorter than this (metres)
    """
    grid = np.asarray(grid)
    clearance_cells = int(np.ceil(clearance / resolution))
    mask = frontier_mask(grid, clearance_cells, free_threshold, occupied_threshold)

    frontiers = []
    for cells in cluster_cells(mask):
        size = len(cells) * resolution
        if size < min_size:
            continue
        # Goal = the cluster's own cell closest to its centre, so it is always a free cell
        centre = cells.mean(axis=0)
        goal = cells[np.argmin(np.sum((cells - centre) ** 2, axis=1))]
        to_world = lambda rc: (origin_x + (rc[..., 1] + 0.5) * resolution,
                               origin_y + (rc[..., 0] + 0.5) * resolution)
        gx, gy = to_world(goal)
        wx, wy = to_world(cells)
        frontiers.append(Frontier(float(gx), float(gy), size, np.column_stack([wx, wy])))
    return frontiers
