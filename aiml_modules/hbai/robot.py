"""Holonomic robot motion and a (deliberately imperfect) odometry model."""
import numpy as np

RADIUS = 0.08       # m, same as robot_radius in Nav2
MAX_SPEED = 0.15    # m/s, same as Nav2's limits for the real robot


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def step(pose, cmd, dt):
    """Move a holonomic robot. cmd = (vx, vy, wz) in the ROBOT frame."""
    x, y, theta = pose
    vx, vy, wz = cmd
    c, s = np.cos(theta), np.sin(theta)
    return np.array([x + (vx * c - vy * s) * dt,
                     y + (vx * s + vy * c) * dt,
                     normalize_angle(theta + wz * dt)])


def relative(p0, p1):
    """Motion from pose p0 to pose p1, expressed in p0's frame: (dx, dy, dtheta)."""
    c, s = np.cos(p0[2]), np.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, normalize_angle(p1[2] - p0[2])])


def compose(pose, delta):
    """Apply a robot-frame motion `delta` to `pose` (the inverse of relative)."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return np.array([pose[0] + c * delta[0] - s * delta[1],
                     pose[1] + s * delta[0] + c * delta[1],
                     normalize_angle(pose[2] + delta[2])])


class DriftingOdometry:
    """Wheel odometry with the errors real omni wheels have.

    scale_error   wheels a bit bigger/smaller than configured (distance is off by this fraction)
    yaw_drift     heading error per metre driven (rollers slip when strafing)
    noise         random error per step
    """

    def __init__(self, start, scale_error=0.04, yaw_drift=0.06, noise=0.003, seed=None):
        self.pose = np.array(start, dtype=float)
        self.scale_error = scale_error
        self.yaw_drift = yaw_drift
        self.noise = noise
        self.rng = np.random.default_rng(seed)

    def update(self, true_prev, true_now):
        """Feed the TRUE motion; get back what the wheels think happened."""
        d = relative(true_prev, true_now)
        dist = np.hypot(d[0], d[1])
        measured = d * [1 + self.scale_error, 1 + self.scale_error, 1.0]
        measured[2] += self.yaw_drift * dist
        measured += self.rng.normal(0, self.noise, 3) * [dist, dist, dist + abs(d[2])]
        self.pose = compose(self.pose, measured)
        return self.pose.copy()
