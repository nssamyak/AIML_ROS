"""Unit tests for the kiwi kinematics. Run with:  colcon test --packages-select hb_control
or directly:  python3 -m pytest src/hb_control/test
"""
import math

import numpy as np
import pytest

from hb_control.kinematics import (KiwiKinematics, integrate_pose, limit_wheel_speeds,
                                   normalize_angle)

R, L = 0.019, 0.052
ANGLES = [math.radians(a) for a in (60, -60, 180)]


@pytest.fixture
def kin():
    return KiwiKinematics(R, L, ANGLES)


def test_round_trip(kin):
    for twist in [(0.1, 0, 0), (0, 0.1, 0), (0, 0, 1.0), (0.12, -0.07, 0.4)]:
        wheels = kin.body_to_wheels(*twist)
        assert np.allclose(kin.wheels_to_body(wheels), twist, atol=1e-12)


def test_forward_leaves_back_wheel_still(kin):
    left, right, back = kin.body_to_wheels(0.1, 0.0, 0.0)
    assert back == pytest.approx(0.0, abs=1e-12)
    # front wheels turn equally but opposite ways
    assert left == pytest.approx(-right)


def test_pure_rotation_turns_all_wheels_equally(kin):
    wheels = kin.body_to_wheels(0.0, 0.0, 1.0)
    assert np.allclose(wheels, L / R)


def test_strafe_left(kin):
    left, right, back = kin.body_to_wheels(0.0, 0.1, 0.0)
    # back wheel rolls along -y when the robot moves +y ... so it spins negative
    assert back == pytest.approx(-0.1 / R)
    assert left == pytest.approx(right)


def test_limit_keeps_direction():
    w = np.array([10.0, -40.0, 20.0])
    limited = limit_wheel_speeds(w, 20.0)
    assert np.max(np.abs(limited)) == pytest.approx(20.0)
    assert np.allclose(limited / limited[1], w / w[1])
    assert np.allclose(limit_wheel_speeds(w, 100.0), w)


def test_integrate_straight_and_circle():
    x, y, th = integrate_pose(0, 0, 0, 1.0, 0.0, 0.0)
    assert (x, y, th) == pytest.approx((1.0, 0.0, 0.0))

    # drive a full circle of radius 1 m in many small steps -> back at start
    x = y = th = 0.0
    steps = 1000
    ds = 2 * math.pi / steps
    for _ in range(steps):
        x, y, th = integrate_pose(x, y, th, ds, 0.0, ds)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_normalize_angle():
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)


def test_rejects_bad_layout():
    with pytest.raises(ValueError):
        KiwiKinematics(R, L, [0.0, 0.0, 0.0])
