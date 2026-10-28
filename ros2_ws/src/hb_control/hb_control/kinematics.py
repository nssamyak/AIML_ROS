"""Kinematics of an omni-wheel (kiwi) drive. Pure Python + numpy, no ROS.

Each wheel i sits at angle theta_i around the robot centre, a distance L
from it, and its axle points at the centre. An omni wheel can only push
along its rolling direction (the tangent of the circle); its rollers let
it slide freely sideways.

The speed of the robot at wheel i, projected on the rolling direction
(-sin theta_i, cos theta_i), must equal the wheel's rim speed r * w_i:

    r * w_i = -sin(theta_i) * vx + cos(theta_i) * vy + L * wz

Stack the three wheels and you get   w = J @ [vx, vy, wz]   (body -> wheels).
Inverting J gives                    [vx, vy, wz] = J^-1 @ w (wheels -> body).

The same code works for 4-wheel X-drive omni robots: just pass 4 angles
(the pseudo-inverse handles the extra wheel).
"""
import math

import numpy as np


class KiwiKinematics:

    def __init__(self, wheel_radius, base_radius, wheel_angles):
        """
        wheel_radius  wheel radius in m
        base_radius   robot centre to wheel contact point in m
        wheel_angles  wheel positions in radians, CCW from the robot's +x axis
        """
        if wheel_radius <= 0 or base_radius <= 0:
            raise ValueError('wheel_radius and base_radius must be positive')
        if len(wheel_angles) < 3:
            raise ValueError('need at least 3 omni wheels for holonomic motion')

        self.wheel_radius = wheel_radius
        self.base_radius = base_radius
        self.wheel_angles = list(wheel_angles)

        rows = [[-math.sin(t), math.cos(t), base_radius] for t in wheel_angles]
        self.J = np.array(rows) / wheel_radius     # body twist  -> wheel speeds
        self.J_inv = np.linalg.pinv(self.J)        # wheel speeds -> body twist

        if np.linalg.matrix_rank(self.J) < 3:
            raise ValueError('wheel layout cannot move in every direction')

    def body_to_wheels(self, vx, vy, wz):
        """Robot velocity (m/s, m/s, rad/s) -> wheel speeds (rad/s)."""
        return self.J @ np.array([vx, vy, wz], dtype=float)

    def wheels_to_body(self, wheel_speeds):
        """Wheel speeds (rad/s) or wheel angle changes (rad) -> robot velocity or displacement."""
        vx, vy, wz = self.J_inv @ np.asarray(wheel_speeds, dtype=float)
        return float(vx), float(vy), float(wz)


def limit_wheel_speeds(wheel_speeds, max_speed):
    """Scale all wheels down together so none exceeds max_speed.

    Scaling them together (instead of clipping each one) keeps the robot
    moving in the requested direction, only slower.
    """
    w = np.asarray(wheel_speeds, dtype=float)
    peak = float(np.max(np.abs(w))) if w.size else 0.0
    if max_speed > 0 and peak > max_speed:
        w = w * (max_speed / peak)
    return w


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def integrate_pose(x, y, theta, dx, dy, dtheta):
    """Add a small body-frame move (dx, dy, dtheta) to a world pose.

    Uses the heading halfway through the move, which is much more accurate
    than the heading at the start when the robot turns while driving.
    """
    mid = theta + dtheta / 2.0
    x += dx * math.cos(mid) - dy * math.sin(mid)
    y += dx * math.sin(mid) + dy * math.cos(mid)
    return x, y, normalize_angle(theta + dtheta)
