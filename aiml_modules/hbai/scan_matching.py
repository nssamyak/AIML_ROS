"""Correlative scan matching: the front end of slam_toolbox (Karto).

Odometry says "you are roughly HERE". We try many poses around that guess
and keep the one where the new scan lines up best with the map built so far.

Score of a pose = average "wall-ness" of the map at the scan's end points,
where wall-ness comes from a LIKELIHOOD FIELD: the occupied cells blurred
with a Gaussian, so a point 2 cm off a wall still gets a good score.

slam_toolbox does the same search (coarse grid then fine grid, see
correlation_search_space_* and the *_variance_penalty parameters), then
adds loop closure and graph optimisation on top.
"""
import numpy as np


def gaussian_blur(image, sigma_cells):
    radius = max(1, int(3 * sigma_cells))
    k = np.exp(-0.5 * (np.arange(-radius, radius + 1) / sigma_cells) ** 2)
    k /= k.sum()
    out = np.apply_along_axis(lambda row: np.convolve(row, k, mode='same'), 1, image)
    return np.apply_along_axis(lambda col: np.convolve(col, k, mode='same'), 0, out)


def likelihood_field(grid, sigma=0.05):
    """1.0 on walls, fading to 0 about 2*sigma away from them."""
    field = gaussian_blur(grid.occupied().astype(float), sigma / grid.geom.res)
    return field / max(field.max(), 1e-9)


def score_poses(field, geom, points, poses):
    """Mean field value at the scan points (robot frame, (K, 2)) for each candidate pose (N, 3)."""
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    wx = poses[:, 0:1] + c * points[None, :, 0] - s * points[None, :, 1]
    wy = poses[:, 1:2] + s * points[None, :, 0] + c * points[None, :, 1]
    return geom.lookup(field, np.stack([wx, wy], axis=-1)).mean(axis=1)


def _candidates(centre, half_xy, step_xy, half_th, step_th):
    dxy = np.arange(-half_xy, half_xy + 1e-9, step_xy)
    dth = np.arange(-half_th, half_th + 1e-9, step_th)
    gx, gy, gt = np.meshgrid(dxy, dxy, dth, indexing='ij')
    return np.stack([centre[0] + gx.ravel(), centre[1] + gy.ravel(), centre[2] + gt.ravel()], axis=1)


def match(field, geom, points, guess, search_xy=0.15, search_th=0.15,
          penalty_xy=0.25, penalty_th=0.25):
    """Best pose for `points` (robot frame) near `guess`. Returns (pose, score).

    The penalty slightly prefers poses close to the odometry guess, like
    slam_toolbox's distance_variance_penalty / angle_variance_penalty.
    """
    guess = np.asarray(guess, dtype=float)

    def best_of(cands):
        score = score_poses(field, geom, points, cands)
        d_xy = np.hypot(cands[:, 0] - guess[0], cands[:, 1] - guess[1])
        d_th = np.abs(cands[:, 2] - guess[2])
        score = score - penalty_xy * d_xy ** 2 - penalty_th * d_th ** 2
        i = int(np.argmax(score))
        return cands[i], score[i]

    coarse, _ = best_of(_candidates(guess, search_xy, 0.025, search_th, 0.025))
    fine, score = best_of(_candidates(coarse, 0.025, 0.005, 0.025, 0.005))
    return fine, float(score)
