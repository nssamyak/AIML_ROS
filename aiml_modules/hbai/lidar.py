"""2D lidar model, roughly an RPLidar A1/C1: 360 beams, 0.12 - 8 m, ~1 cm noise.

Same numbers as the Gazebo sensor in hb_description/urdf/hb_bot.gazebo.xacro.
"""
import numpy as np


class Lidar:

    def __init__(self, n_beams=360, min_range=0.12, max_range=8.0, noise_std=0.01, seed=None):
        self.n_beams = n_beams
        self.min_range = min_range
        self.max_range = max_range
        self.noise_std = noise_std
        # Beam angles in the ROBOT frame, starting at -180 deg, counter-clockwise
        self.angles = -np.pi + np.arange(n_beams) * 2 * np.pi / n_beams
        self.rng = np.random.default_rng(seed)

    def scan(self, world, pose):
        """Ranges (m) for every beam; inf where nothing was hit (too far or too close)."""
        x, y, theta = pose
        ranges = world.raycast((x, y), theta + self.angles, self.max_range)
        hit = np.isfinite(ranges)
        ranges[hit] += self.rng.normal(0.0, self.noise_std, hit.sum())
        ranges[ranges < self.min_range] = np.inf
        return ranges


def scan_to_points(ranges, angles, pose=None):
    """Polar scan -> (N, 2) points of the beams that hit something.

    pose=None: points in the ROBOT frame (x forward, y left).
    pose=(x, y, theta): points in the WORLD frame.
    """
    hit = np.isfinite(ranges)
    r, a = ranges[hit], angles[hit]
    pts = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
    if pose is None:
        return pts
    return transform_points(pts, pose)


def transform_points(pts, pose):
    """Rotate by theta then shift by (x, y): robot frame -> world frame."""
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    return pts @ np.array([[c, -s], [s, c]]).T + [x, y]
