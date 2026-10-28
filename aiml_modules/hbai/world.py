"""The 2D world: the same arena as hb_gazebo/worlds/arena.sdf.

Obstacles are boxes (possibly rotated) and cylinders. Ray casting is exact:
every lidar beam is intersected with every box edge and every circle.
"""
import numpy as np

# Robot spawn pose in Gazebo: x, y, heading
START = (-1.3, -0.2, 0.0)

# Inside of the outer walls
ARENA_X = (-2.0, 2.0)
ARENA_Y = (-1.5, 1.5)


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def box_corners(cx, cy, sx, sy, yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    local = np.array([[-sx / 2, -sy / 2], [sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2]])
    return local @ np.array([[c, -s], [s, c]]).T + [cx, cy]


class World:
    """Static obstacles. boxes: rows of (cx, cy, size_x, size_y, yaw). circles: rows of (cx, cy, r)."""

    def __init__(self, boxes, circles, xlim, ylim):
        self.boxes = np.array(boxes, dtype=float).reshape(-1, 5)
        self.circles = np.array(circles, dtype=float).reshape(-1, 3)
        self.xlim, self.ylim = tuple(xlim), tuple(ylim)
        segments = []
        for b in self.boxes:
            c = box_corners(*b)
            segments += [[*c[i], *c[(i + 1) % 4]] for i in range(4)]
        self.segments = np.array(segments).reshape(-1, 4)

    # ------------------------------------------------------------------ lidar
    def raycast(self, origin, angles, max_range=np.inf):
        """Distance along each ray (angles in world frame) to the first obstacle; inf if none."""
        o = np.asarray(origin, dtype=float)[:2]
        d = np.stack([np.cos(angles), np.sin(angles)], axis=-1)            # (R, 2)
        best = np.full(len(d), np.inf)

        if len(self.segments):
            # Solve  o + t*d = a + u*e   for every ray / segment pair
            a = self.segments[:, :2]
            e = self.segments[:, 2:] - a                                    # (N, 2)
            w = a - o
            denom = _cross(d[:, None, :], e[None])                          # (R, N)
            with np.errstate(divide='ignore', invalid='ignore'):
                t = _cross(w, e)[None] / denom
                u = _cross(w[None], d[:, None, :]) / denom
            ok = (np.abs(denom) > 1e-12) & (t >= 0) & (u >= 0) & (u <= 1)
            best = np.minimum(best, np.where(ok, t, np.inf).min(axis=1))

        if len(self.circles):
            f = o - self.circles[:, :2]                                      # (C, 2)
            b = d @ f.T                                                      # (R, C)
            c = (f ** 2).sum(axis=1) - self.circles[:, 2] ** 2
            disc = b ** 2 - c
            with np.errstate(invalid='ignore'):
                t = -b - np.sqrt(disc)
            ok = (disc >= 0) & (t >= 0)
            best = np.minimum(best, np.where(ok, t, np.inf).min(axis=1))

        best[best > max_range] = np.inf
        return best

    # ------------------------------------------------------------------ geometry
    def inside(self, points):
        """True for points inside any obstacle."""
        p = np.atleast_2d(np.asarray(points, dtype=float))[:, :2]
        inside = np.zeros(len(p), dtype=bool)
        for cx, cy, sx, sy, yaw in self.boxes:
            c, s = np.cos(yaw), np.sin(yaw)
            q = (p - [cx, cy]) @ np.array([[c, -s], [s, c]])
            inside |= (np.abs(q[:, 0]) <= sx / 2) & (np.abs(q[:, 1]) <= sy / 2)
        for cx, cy, r in self.circles:
            inside |= np.hypot(p[:, 0] - cx, p[:, 1] - cy) <= r
        return inside

    def distance(self, points):
        """Distance from each point to the nearest obstacle surface (0 inside obstacles)."""
        p = np.atleast_2d(np.asarray(points, dtype=float))[:, :2]
        d = np.full(len(p), np.inf)
        for start in range(0, len(p), 4096):                     # chunks keep memory small
            q = p[start:start + 4096]
            dq = np.full(len(q), np.inf)
            if len(self.segments):
                a, b = self.segments[:, :2], self.segments[:, 2:]
                ab = b - a
                ap = q[:, None, :] - a[None]
                t = np.clip((ap * ab).sum(-1) / (ab * ab).sum(-1), 0, 1)
                dq = np.minimum(dq, np.linalg.norm(ap - t[..., None] * ab, axis=-1).min(axis=1))
            if len(self.circles):
                dc = np.linalg.norm(q[:, None, :] - self.circles[None, :, :2], axis=-1) - self.circles[:, 2]
                dq = np.minimum(dq, dc.min(axis=1))
            d[start:start + 4096] = dq
        d[self.inside(p)] = 0.0
        return np.maximum(d, 0.0)

    def occupancy(self, geom, thickness=0.6):
        """Rasterise the obstacles onto a grid (True = occupied)."""
        centres = geom.centers().reshape(-1, 2)
        occ = self.distance(centres) <= thickness * geom.res
        return occ.reshape(geom.shape)

    def random_free_point(self, rng, clearance=0.2):
        while True:
            p = rng.uniform([ARENA_X[0], ARENA_Y[0]], [ARENA_X[1], ARENA_Y[1]])
            if self.distance(p)[0] > clearance:
                return p


def arena():
    """The workshop arena: 4 m x 3 m, three rooms, two boxes, two cylinders."""
    boxes = [
        (0.0, 1.525, 4.1, 0.05, 0.0),      # outer walls
        (0.0, -1.525, 4.1, 0.05, 0.0),
        (2.025, 0.0, 0.05, 3.0, 0.0),
        (-2.025, 0.0, 0.05, 3.0, 0.0),
        (-0.6, -0.4, 0.05, 2.2, 0.0),      # inner walls
        (1.2, 0.2, 1.6, 0.05, 0.0),
        (1.0, -1.05, 0.05, 0.9, 0.0),
        (0.0, -0.6, 0.3, 0.3, 0.3),        # boxes
        (-1.5, -1.0, 0.25, 0.4, 0.0),
    ]
    circles = [(-1.3, 0.9, 0.15), (1.4, 0.9, 0.12)]
    return World(boxes, circles, xlim=(-2.3, 2.3), ylim=(-1.8, 1.8))


def inside_arena(points):
    p = np.atleast_2d(points)
    return (p[:, 0] > ARENA_X[0]) & (p[:, 0] < ARENA_X[1]) & (p[:, 1] > ARENA_Y[0]) & (p[:, 1] < ARENA_Y[1])
