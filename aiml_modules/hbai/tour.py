"""A fixed drive around the arena, used by the mapping and SLAM modules.

The robot visits every room, turning to face where it's going (so heading
errors in odometry matter, like on the real robot).
"""
import numpy as np

from hbai import robot
from hbai.gridmap import GridGeom
from hbai.planning import costmap, plan
from hbai.world import START

TOUR = [(-1.3, 0.6), (-0.2, 1.1), (1.7, 0.6), (1.7, -0.2), (1.5, -1.1), (0.4, -1.1), (-0.2, -0.2)]


def tour_poses(world, waypoints=TOUR, speed=0.2, dt=0.1, start=START, lookahead=0.25):
    """True robot poses (N, 3), one per time step, driving through all waypoints."""
    geom = GridGeom(world.xlim, world.ylim, 0.05)
    cost = costmap(world.occupancy(geom), geom.res, inflation_radius=0.35)
    pose = np.array(start, dtype=float)
    poses = [pose.copy()]
    for wp in waypoints:
        path, _ = plan(geom, cost, pose, wp)
        if path is None:
            continue
        path = np.vstack([path, wp])
        for _ in range(5000):
            if np.hypot(*(pose[:2] - wp)) < 0.05:
                break
            i = int(np.argmin(np.linalg.norm(path - pose[:2], axis=1)))
            ahead = np.cumsum(np.r_[0, np.linalg.norm(np.diff(path[i:], axis=0), axis=1)])
            target = path[min(i + int(np.searchsorted(ahead, lookahead)), len(path) - 1)]
            v = target - pose[:2]
            v = v / max(np.linalg.norm(v), 1e-9) * min(speed, np.linalg.norm(v) + 0.02)
            heading_error = robot.normalize_angle(np.arctan2(v[1], v[0]) - pose[2])
            wz = np.clip(2.0 * heading_error, -1.0, 1.0)
            c, s = np.cos(pose[2]), np.sin(pose[2])
            pose = robot.step(pose, (c * v[0] + s * v[1], -s * v[0] + c * v[1], wz), dt)
            poses.append(pose.copy())
    return np.array(poses)
