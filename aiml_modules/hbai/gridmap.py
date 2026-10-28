"""Grid geometry: converting between metres and grid cells.

Same convention as ROS nav_msgs/OccupancyGrid: row 0 is the BOTTOM of the
map (smallest y), column 0 is the left (smallest x). So images are drawn
with origin='lower'.
"""
import numpy as np


class GridGeom:

    def __init__(self, xlim, ylim, res):
        self.res = float(res)
        self.x0, self.y0 = float(xlim[0]), float(ylim[0])
        self.width = int(round((xlim[1] - xlim[0]) / res))
        self.height = int(round((ylim[1] - ylim[0]) / res))
        self.shape = (self.height, self.width)

    def to_cell(self, xy):
        """World points (..., 2) -> (rows, cols) integer arrays."""
        xy = np.asarray(xy, dtype=float)
        cols = np.floor((xy[..., 0] - self.x0) / self.res).astype(int)
        rows = np.floor((xy[..., 1] - self.y0) / self.res).astype(int)
        return rows, cols

    def in_bounds(self, rows, cols):
        return (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)

    def to_world(self, rows, cols):
        """Cell centres in metres, shape (..., 2)."""
        return np.stack([self.x0 + (np.asarray(cols) + 0.5) * self.res,
                         self.y0 + (np.asarray(rows) + 0.5) * self.res], axis=-1)

    def centers(self):
        rows, cols = np.mgrid[0:self.height, 0:self.width]
        return self.to_world(rows, cols)

    def lookup(self, array, xy, outside=0.0):
        """Value of `array` at world points xy (..., 2); `outside` for points off the grid."""
        xy = np.asarray(xy, dtype=float)
        rows, cols = self.to_cell(xy)
        ok = self.in_bounds(rows, cols)
        out = np.full(rows.shape, outside, dtype=float)
        out[ok] = array[rows[ok], cols[ok]]
        return out

    @property
    def extent(self):
        """For matplotlib imshow(..., origin='lower', extent=geom.extent)."""
        return (self.x0, self.x0 + self.width * self.res, self.y0, self.y0 + self.height * self.res)
