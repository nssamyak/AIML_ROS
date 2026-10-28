"""Occupancy grid mapping: a Bayes filter per cell, stored as log-odds.

Each cell holds the belief "this cell is occupied". Every lidar beam is
evidence:
  - cells the beam passed THROUGH are probably free     -> add l_free (< 0)
  - the cell where the beam ENDED is probably occupied   -> add l_occ  (> 0)

Why log-odds? Bayes' rule for a binary state turns into plain addition:
    l = log(p / (1 - p))      l_new = l_old + l_measurement
so mapping is just counting evidence, and p = 1 / (1 + exp(-l)) at the end.
"""
import numpy as np


class OccupancyGrid:

    def __init__(self, geom, l_occ=0.85, l_free=-0.4, l_min=-4.0, l_max=4.0):
        self.geom = geom
        self.l_occ, self.l_free = l_occ, l_free
        self.l_min, self.l_max = l_min, l_max
        self.logodds = np.zeros(geom.shape)
        self.seen = np.zeros(geom.shape, dtype=bool)

    def update(self, pose, angles, ranges, max_range=4.0):
        """Add one scan taken at `pose` (angles in the robot frame)."""
        g = self.geom
        x, y, theta = pose
        hit = np.isfinite(ranges) & (ranges <= max_range)
        if not hit.any():
            return
        a = theta + angles[hit]
        r = ranges[hit]
        ca, sa = np.cos(a), np.sin(a)

        # Free space: sample every half cell along each beam, stopping just short of the hit
        step = g.res / 2
        s = np.arange(int(r.max() / step) + 1) * step
        along = s[None, :] < (r[:, None] - g.res / 2)
        px = x + s[None, :] * ca[:, None]
        py = y + s[None, :] * sa[:, None]
        free = self._cell_ids(px[along], py[along])

        # Occupied: the cell at the end of each beam
        occ = self._cell_ids(x + r * ca, y + r * sa)
        free = np.setdiff1d(free, occ)

        flat = self.logodds.reshape(-1)
        flat[free] += self.l_free
        flat[occ] += self.l_occ
        np.clip(flat, self.l_min, self.l_max, out=flat)
        self.seen.reshape(-1)[free] = True
        self.seen.reshape(-1)[occ] = True

    def _cell_ids(self, xs, ys):
        rows, cols = self.geom.to_cell(np.stack([xs, ys], axis=-1))
        ok = self.geom.in_bounds(rows, cols)
        return np.unique(rows[ok] * self.geom.width + cols[ok])

    # ------------------------------------------------------------------ views
    def probability(self):
        return 1.0 / (1.0 + np.exp(-self.logodds))

    def occupied(self, threshold=0.65):
        return self.seen & (self.probability() > threshold)

    def free(self, threshold=0.5):
        """Seen, and more evidence for free than for occupied."""
        return self.seen & (self.probability() < threshold)

    def to_ros(self):
        """nav_msgs/OccupancyGrid values, the way slam_toolbox publishes them:
        -1 unknown, 0 free, 100 occupied (50 for the rare undecided cell)."""
        out = np.full(self.geom.shape, 50, dtype=np.int16)
        out[self.free()] = 0
        out[self.occupied()] = 100
        out[~self.seen] = -1
        return out
